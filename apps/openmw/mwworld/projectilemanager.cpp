#include "projectilemanager.hpp"

#include <algorithm>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>

#include <osg/PositionAttitudeTransform>

#include <components/debug/debuglog.hpp>

#include <components/esm3/actoridconverter.hpp>
#include <components/esm3/esmreader.hpp>
#include <components/esm3/esmwriter.hpp>
#include <components/esm3/loadench.hpp>
#include <components/esm3/loadmgef.hpp>
#include <components/esm3/loadrace.hpp>
#include <components/esm3/projectilestate.hpp>

#include <components/esm/quaternion.hpp>
#include <components/esm/util.hpp>
#include <components/esm/vector3.hpp>

#include <components/misc/constants.hpp>
#include <components/misc/convert.hpp>
#include <components/misc/resourcehelpers.hpp>
#include <components/misc/strings/lower.hpp>

#include <components/resource/resourcesystem.hpp>
#include <components/resource/scenemanager.hpp>

#include <components/sceneutil/controller.hpp>
#include <components/sceneutil/lightmanager.hpp>
#include <components/sceneutil/nodecallback.hpp>
#include <components/sceneutil/visitor.hpp>

#include <components/settings/values.hpp>

#include "../mwworld/class.hpp"
#include "../mwworld/esmstore.hpp"
#include "../mwworld/inventorystore.hpp"
#include "../mwworld/manualref.hpp"
#include "../mwworld/worldmodel.hpp"

#include "../mwbase/environment.hpp"
#include "../mwbase/luamanager.hpp"
#include "../mwbase/soundmanager.hpp"
#include "../mwbase/windowmanager.hpp"
#include "../mwbase/world.hpp"

#include "../mwmechanics/actorutil.hpp"
#include "../mwmechanics/combat.hpp"
#include "../mwmechanics/creaturestats.hpp"
#include "../mwmechanics/spellcasting.hpp"

#include "../mwrender/animation.hpp"
#include "../mwrender/renderingmanager.hpp"
#include "../mwrender/util.hpp"
#include "../mwrender/vismask.hpp"

#include "../mwsound/sound.hpp"

#include "../mwphysics/physicssystem.hpp"
#include "../mwphysics/projectile.hpp"

namespace
{
    ESM::EffectList getMagicBoltData(std::vector<ESM::RefId>& projectileIDs, std::set<ESM::RefId>& sounds, float& speed,
        VFS::Path::NormalizedView& texture, std::string& sourceName, const ESM::RefId& id)
    {
        const MWWorld::ESMStore& esmStore = *MWBase::Environment::get().getESMStore();
        const ESM::EffectList* effects;
        if (const ESM::Spell* spell = esmStore.get<ESM::Spell>().search(id)) // check if it's a spell
        {
            sourceName = spell->mName;
            effects = &spell->mEffects;
        }
        else // check if it's an enchanted item
        {
            MWWorld::ManualRef ref(esmStore, id);
            MWWorld::Ptr ptr = ref.getPtr();
            const ESM::Enchantment* ench = esmStore.get<ESM::Enchantment>().find(ptr.getClass().getEnchantment(ptr));
            sourceName = ptr.getClass().getName(ptr);
            effects = &ench->mEffects;
        }

        int count = 0;
        speed = 0.0f;
        ESM::EffectList projectileEffects;
        for (const ESM::IndexedENAMstruct& effect : effects->mList)
        {
            const ESM::MagicEffect* magicEffect
                = MWBase::Environment::get().getESMStore()->get<ESM::MagicEffect>().find(effect.mData.mEffectID);

            // Speed of multi-effect projectiles should be the average of the constituent effects,
            // based on observation of the original engine.
            speed += magicEffect->mData.mSpeed;
            count++;

            if (effect.mData.mRange != ESM::RT_Target)
                continue;

            if (magicEffect->mBolt.empty())
                projectileIDs.emplace_back(ESM::RefId::stringRefId("VFX_DefaultBolt"));
            else
                projectileIDs.push_back(magicEffect->mBolt);

            if (!magicEffect->mBoltSound.empty())
                sounds.emplace(magicEffect->mBoltSound);
            else
                sounds.emplace(MWBase::Environment::get()
                                   .getESMStore()
                                   ->get<ESM::Skill>()
                                   .find(magicEffect->mData.mSchool)
                                   ->mSchool->mBoltSound);
            projectileEffects.mList.push_back(effect);
        }

        if (count != 0)
            speed /= count;

        // the particle texture is only used if there is only one projectile
        if (projectileEffects.mList.size() == 1)
        {
            const ESM::MagicEffect* magicEffect
                = MWBase::Environment::get().getESMStore()->get<ESM::MagicEffect>().find(
                    effects->mList.begin()->mData.mEffectID);
            texture = magicEffect->mParticle.getNormalized();
        }

        // insert a VFX_Multiple projectile if there are multiple projectile effects
        if (projectileEffects.mList.size() > 1)
        {
            const ESM::RefId projectileId
                = ESM::RefId::stringRefId("VFX_Multiple" + std::to_string(effects->mList.size()));
            projectileIDs.insert(projectileIDs.begin(), projectileId);
        }

        return projectileEffects;
    }

    osg::Vec4 getMagicBoltLightDiffuseColor(const ESM::EffectList& effects)
    {
        // Calculate combined light diffuse color from magical effects
        osg::Vec4 lightDiffuseColor;
        for (const ESM::IndexedENAMstruct& enam : effects.mList)
        {
            const ESM::MagicEffect* magicEffect
                = MWBase::Environment::get().getESMStore()->get<ESM::MagicEffect>().find(enam.mData.mEffectID);
            lightDiffuseColor += magicEffect->getColor();
        }
        size_t numberOfEffects = effects.mList.size();
        lightDiffuseColor /= static_cast<float>(numberOfEffects);

        return lightDiffuseColor;
    }

    // Whether an arrow would stick in a surface (wood, plants, earth) or glance off it (stone, metal), judging by
    // its texture and model names. Hard materials win when names mention both.
    bool isSoftSurface(std::string_view texture, std::string_view model, bool unknownIsSoft)
    {
        static constexpr std::string_view hard[] = { "stone", "rock", "brick", "cobble", "marble", "granite", "slate",
            "metal", "iron", "steel", "bronze", "silver", "gold", "dwrv", "dwem", "glass", "crystal", "tile", "plaster",
            "stucco", "chitin", "bone", "shell", "pave", "ice", "lava", "velothi", "daed", "ruin" };
        static constexpr std::string_view soft[] = { "wood", "wd_", "bark", "tree", "log", "plank", "board", "timber",
            "branch", "root", "crate", "barrel", "basket", "wicker", "straw", "thatch", "hay", "fabric", "cloth",
            "banner", "rug", "carpet", "tapestry", "rope", "leather", "hide", "fur", "mushroom", "shroom", "fung",
            "flora", "moss", "dirt", "mud", "sand", "grass", "soil", "ground", "kelp" };
        const auto classify = [](std::string_view name) -> std::optional<bool> {
            const std::string lower = Misc::StringUtils::lowerCase(name);
            for (const std::string_view word : hard)
                if (lower.find(word) != std::string::npos)
                    return false;
            for (const std::string_view word : soft)
                if (lower.find(word) != std::string::npos)
                    return true;
            return std::nullopt;
        };
        if (const std::optional<bool> fromTexture = classify(texture))
            return *fromTexture;
        if (const std::optional<bool> fromModel = classify(model))
            return *fromModel;
        return unknownIsSoft;
    }

    osg::Quat lookAt(const osg::Vec3f& pos)
    {
        // Rotate the forward vector towards the position (used for gravity-affected projectiles)
        // Can't use Quat::makeRotate as the shortest angle contains undesirable local roll
        const float dist = pos.length();
        if (dist < 1e-4f)
            return {};

        const osg::Vec3f dir = pos / dist;
        osg::Vec3f right = dir ^ osg::Z_AXIS;
        if (right.normalize() < 1e-4f)
            right = osg::X_AXIS;

        const osg::Vec3f up = right ^ dir;

        osg::Matrixf mat(right.x(), right.y(), right.z(), 0.f, dir.x(), dir.y(), dir.z(), 0.f, up.x(), up.y(), up.z(),
            0.f, 0.f, 0.f, 0.f, 1.f);

        osg::Quat orient;
        orient.set(mat);
        return orient;
    }
}

namespace MWWorld
{

    ProjectileManager::ProjectileManager(osg::Group* parent, Resource::ResourceSystem* resourceSystem,
        MWRender::RenderingManager* rendering, MWPhysics::PhysicsSystem* physics)
        : mParent(parent)
        , mResourceSystem(resourceSystem)
        , mRendering(rendering)
        , mPhysics(physics)
        , mCleanupTimer(0.0f)
    {
    }

    /// Rotates an osg::PositionAttitudeTransform over time.
    class RotateCallback : public SceneUtil::NodeCallback<RotateCallback, osg::PositionAttitudeTransform*>
    {
    public:
        RotateCallback(const osg::Vec3f& axis = osg::Vec3f(0, -1, 0), float rotateSpeed = osg::PI * 2)
            : mAxis(axis)
            , mRotateSpeed(rotateSpeed)
        {
        }

        void operator()(osg::PositionAttitudeTransform* node, osg::NodeVisitor* nv)
        {
            double time = nv->getFrameStamp()->getSimulationTime();

            osg::Quat orient = osg::Quat(time * mRotateSpeed, mAxis);
            node->setAttitude(orient);

            traverse(node, nv);
        }

    private:
        osg::Vec3f mAxis;
        float mRotateSpeed;
    };

    void ProjectileManager::createModel(State& state, VFS::Path::NormalizedView model, const osg::Vec3f& pos,
        const osg::Quat& orient, bool rotate, bool createLight, osg::Vec4 lightDiffuseColor,
        VFS::Path::NormalizedView texture)
    {
        state.mNode = new osg::PositionAttitudeTransform;
        state.mNode->setNodeMask(MWRender::Mask_Effect);
        state.mNode->setPosition(pos);
        state.mNode->setAttitude(orient);

        osg::Group* attachTo = state.mNode;

        if (rotate)
        {
            osg::ref_ptr<osg::PositionAttitudeTransform> rotateNode(new osg::PositionAttitudeTransform);
            rotateNode->addUpdateCallback(new RotateCallback());
            state.mNode->addChild(rotateNode);
            attachTo = rotateNode;
        }

        osg::ref_ptr<osg::Node> projectile = mResourceSystem->getSceneManager()->getInstance(model, attachTo);

        if (state.mIdMagic.size() > 1)
        {
            for (size_t iter = 1; iter != state.mIdMagic.size(); ++iter)
            {
                std::ostringstream nodeName;
                nodeName << "Dummy" << std::setw(2) << std::setfill('0') << iter;
                const ESM::Weapon* weapon
                    = MWBase::Environment::get().getESMStore()->get<ESM::Weapon>().find(state.mIdMagic.at(iter));
                std::string nameToFind = nodeName.str();
                SceneUtil::FindByNameVisitor findVisitor(nameToFind);
                attachTo->accept(findVisitor);
                if (findVisitor.mFoundNode)
                    mResourceSystem->getSceneManager()->getInstance(
                        Misc::ResourceHelpers::correctMeshPath(weapon->mModel.getNormalized()), findVisitor.mFoundNode);
            }
        }

        if (createLight)
        {
            osg::ref_ptr<SceneUtil::Light> projectileLight(new SceneUtil::Light);
            projectileLight->setAmbient(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
            projectileLight->setDiffuse(lightDiffuseColor);
            projectileLight->setSpecular(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
            projectileLight->setConstantAttenuation(0.f);
            projectileLight->setLinearAttenuation(0.1f);
            projectileLight->setQuadraticAttenuation(0.f);
            projectileLight->setPosition(osg::Vec4(pos, 1.0));

            SceneUtil::LightSource* projectileLightSource = new SceneUtil::LightSource;
            projectileLightSource->setNodeMask(MWRender::Mask_Lighting);
            projectileLightSource->setRadius(66.f);

            state.mNode->addChild(projectileLightSource);
            projectileLightSource->setLight(projectileLight);
        }

        state.mNode->addCullCallback(new SceneUtil::LightListCallback);

        mParent->addChild(state.mNode);

        state.mEffectAnimationTime = std::make_shared<MWRender::EffectAnimationTime>();

        SceneUtil::AssignControllerSourcesVisitor assignVisitor(state.mEffectAnimationTime);
        state.mNode->accept(assignVisitor);

        MWRender::overrideFirstRootTexture(texture, mResourceSystem, *projectile);
    }

    void ProjectileManager::update(State& state, float duration)
    {
        state.mEffectAnimationTime->addTime(duration);
    }

    void ProjectileManager::launchMagicBolt(
        const ESM::RefId& spellId, const Ptr& caster, const osg::Vec3f& fallbackDirection, ESM::RefNum item)
    {
        osg::Vec3f pos = caster.getRefData().getPosition().asVec3();
        if (caster.getClass().isActor())
        {
            // Note: we ignore the collision box offset, this is required to make some flying creatures work as
            // intended.
            pos.z() += mPhysics->getRenderingHalfExtents(caster).z() * 2 * Constants::TorsoHeight;
        }

        // Actors can't cast target spells underwater
        if (caster.getClass().isActor() && MWBase::Environment::get().getWorld()->isUnderwater(caster.getCell(), pos))
            return;

        osg::Quat orient;
        if (caster.getClass().isActor())
            orient = osg::Quat(caster.getRefData().getPosition().rot[0], osg::Vec3f(-1, 0, 0))
                * osg::Quat(caster.getRefData().getPosition().rot[2], osg::Vec3f(0, 0, -1));
        else
            orient.makeRotate(osg::Vec3f(0, 1, 0), osg::Vec3f(fallbackDirection));

        MagicBoltState state;
        state.mSpellId = spellId;
        state.mCasterHandle = caster;
        state.mItem = item;
        MWBase::Environment::get().getWorldModel()->registerPtr(caster);
        state.mCaster = caster.getCellRef().getRefNum();

        VFS::Path::NormalizedView texture;

        state.mEffects = getMagicBoltData(
            state.mIdMagic, state.mSoundIds, state.mSpeed, texture, state.mSourceName, state.mSpellId);

        // Non-projectile should have been removed by getMagicBoltData
        if (state.mEffects.mList.empty())
            return;

        if (!caster.getClass().isActor() && fallbackDirection.length2() <= 0)
        {
            Log(Debug::Warning) << "Unable to launch magic bolt (direction to target is empty)";
            return;
        }

        const MWWorld::ESMStore& esmStore = *MWBase::Environment::get().getESMStore();
        MWWorld::ManualRef ref(esmStore, state.mIdMagic.at(0));
        MWWorld::Ptr ptr = ref.getPtr();

        osg::Vec4 lightDiffuseColor = getMagicBoltLightDiffuseColor(state.mEffects);

        VFS::Path::Normalized model = ptr.getClass().getCorrectedModel(ptr);
        createModel(state, model, pos, orient, true, true, lightDiffuseColor, texture);

        MWBase::SoundManager* sndMgr = MWBase::Environment::get().getSoundManager();
        for (const auto& soundid : state.mSoundIds)
        {
            MWBase::Sound* sound
                = sndMgr->playSound3D(pos, soundid, 1.0f, 1.0f, MWSound::Type::Sfx, MWSound::PlayMode::Loop);
            if (sound)
                state.mSounds.push_back(sound);
        }

        // in case there are multiple effects, the model is a dummy without geometry. Use the second effect for physics
        // shape
        if (state.mIdMagic.size() > 1)
        {
            model = Misc::ResourceHelpers::correctMeshPath(
                esmStore.get<ESM::Weapon>().find(state.mIdMagic[1])->mModel.getNormalized());
        }
        state.mProjectileId = mPhysics->addProjectile(caster, pos, model, true);
        state.mToDelete = false;
        mMagicBolts.push_back(std::move(state));
    }

    void ProjectileManager::launchProjectile(const Ptr& actor, const ConstPtr& projectile, const osg::Vec3f& pos,
        const osg::Quat& orient, const Ptr& bow, float speed, float attackStrength, float attackWindUp)
    {
        ProjectileState state;
        state.mCaster = actor.getCellRef().getRefNum();
        state.mBowId = bow.getCellRef().getRefId();
        state.mVelocity = orient * osg::Vec3f(0, 1, 0) * speed;
        state.mIdArrow = projectile.getCellRef().getRefId();
        state.mCasterHandle = actor;
        state.mAttackStrength = attackStrength;
        state.mAttackWindUp = attackWindUp;

        MWWorld::ManualRef ref(*MWBase::Environment::get().getESMStore(), projectile.getCellRef().getRefId());
        MWWorld::Ptr ptr = ref.getPtr();

        const VFS::Path::Normalized model = ptr.getClass().getCorrectedModel(ptr);
        createModel(state, model, pos, orient, false, false, osg::Vec4(0, 0, 0, 0));
        if (!ptr.getClass().getEnchantment(ptr).empty())
            SceneUtil::addEnchantedGlow(state.mNode, mResourceSystem, ptr.getClass().getEnchantmentColor(ptr));

        state.mProjectileId = mPhysics->addProjectile(actor, pos, model, false);
        state.mToDelete = false;
        mProjectiles.push_back(std::move(state));
    }

    void ProjectileManager::updateCasters()
    {
        for (auto& state : mProjectiles)
        {
            state.mCasterHandle = state.getCaster();
            mPhysics->setCaster(state.mProjectileId, state.mCasterHandle);
        }

        for (auto& state : mMagicBolts)
        {
            if (!state.mCaster.isSet())
                continue;

            state.mCasterHandle = state.getCaster();
            if (state.mCasterHandle.isEmpty())
            {
                Log(Debug::Error) << "Couldn't find caster with ID " << state.mCaster;
                cleanupMagicBolt(state);
                continue;
            }
            mPhysics->setCaster(state.mProjectileId, state.mCasterHandle);
        }
    }

    void ProjectileManager::update(float dt)
    {
        periodicCleanup(dt);
        moveProjectiles(dt);
        moveMagicBolts(dt);
    }

    void ProjectileManager::periodicCleanup(float dt)
    {
        mCleanupTimer -= dt;
        if (mCleanupTimer <= 0.0f)
        {
            mCleanupTimer = 2.0f;

            auto isCleanable = [](const ProjectileManager::State& state) -> bool {
                const float farawayThreshold = 72000.0f;
                osg::Vec3 playerPos = MWMechanics::getPlayer().getRefData().getPosition().asVec3();
                return (state.mNode->getPosition() - playerPos).length2() >= farawayThreshold * farawayThreshold;
            };

            for (auto& projectileState : mProjectiles)
            {
                if (isCleanable(projectileState))
                    cleanupProjectile(projectileState);
            }

            for (auto& magicBoltState : mMagicBolts)
            {
                if (isCleanable(magicBoltState))
                    cleanupMagicBolt(magicBoltState);
            }
        }
    }

    void ProjectileManager::moveMagicBolts(float duration)
    {
        const bool normaliseRaceSpeed = Settings::game().mNormaliseRaceSpeed;
        for (auto& magicBoltState : mMagicBolts)
        {
            if (magicBoltState.mToDelete)
                continue;

            auto* projectile = mPhysics->getProjectile(magicBoltState.mProjectileId);
            if (!projectile->isActive())
                continue;
            // If the actor caster is gone, the magic bolt needs to be removed from the scene during the next frame.
            MWWorld::Ptr caster = magicBoltState.getCaster();
            if (!caster.isEmpty() && caster.getClass().isActor())
            {
                if (caster.getCellRef().getCount() <= 0 || caster.getClass().getCreatureStats(caster).isDead())
                {
                    cleanupMagicBolt(magicBoltState);
                    continue;
                }
            }

            const auto& store = *MWBase::Environment::get().getESMStore();
            osg::Quat orient = magicBoltState.mNode->getAttitude();
            static float fTargetSpellMaxSpeed
                = store.get<ESM::GameSetting>().find("fTargetSpellMaxSpeed")->mValue.getFloat();
            float speed = fTargetSpellMaxSpeed * magicBoltState.mSpeed;
            if (!normaliseRaceSpeed && !caster.isEmpty() && caster.getClass().isNpc())
            {
                const auto npc = caster.get<ESM::NPC>()->mBase;
                const auto race = store.get<ESM::Race>().find(npc->mRace);
                speed *= npc->isMale() ? race->mData.mMaleWeight : race->mData.mFemaleWeight;
            }
            osg::Vec3f direction = orient * osg::Vec3f(0, 1, 0);
            direction.normalize();
            projectile->setVelocity(direction * speed);

            update(magicBoltState, duration);

            for (const auto& sound : magicBoltState.mSounds)
            {
                sound->setVelocity(direction * speed);
            }

            // For AI actors, get combat targets to use in the ray cast. Only those targets will return a positive hit
            // result.
            std::vector<MWWorld::Ptr> targetActors;
            if (!caster.isEmpty() && caster.getClass().isActor() && caster != MWMechanics::getPlayer())
                caster.getClass().getCreatureStats(caster).getAiSequence().getCombatTargets(targetActors);
            projectile->setValidTargets(targetActors);
        }
    }

    void ProjectileManager::moveProjectiles(float duration)
    {
        for (auto& projectileState : mProjectiles)
        {
            if (projectileState.mToDelete)
                continue;

            auto* projectile = mPhysics->getProjectile(projectileState.mProjectileId);
            if (!projectile->isActive())
                continue;
            // gravity constant - must be way lower than the gravity affecting actors, since we're not
            // simulating aerodynamics at all
            projectileState.mVelocity
                -= osg::Vec3f(0, 0, Constants::GravityConst * Constants::UnitsPerMeter * 0.1f) * duration;

            projectile->setVelocity(projectileState.mVelocity);

            projectileState.mNode->setAttitude(lookAt(projectileState.mVelocity));

            update(projectileState, duration);

            MWWorld::Ptr caster = projectileState.getCaster();

            // For AI actors, get combat targets to use in the ray cast. Only those targets will return a positive hit
            // result.
            std::vector<MWWorld::Ptr> targetActors;
            if (!caster.isEmpty() && caster.getClass().isActor() && caster != MWMechanics::getPlayer())
                caster.getClass().getCreatureStats(caster).getAiSequence().getCombatTargets(targetActors);
            projectile->setValidTargets(targetActors);
        }
    }

    void ProjectileManager::processHits()
    {
        for (auto& projectileState : mProjectiles)
        {
            if (projectileState.mToDelete)
                continue;

            auto* projectile = mPhysics->getProjectile(projectileState.mProjectileId);

            const auto pos = projectile->getSimulationPosition();
            projectileState.mNode->setPosition(pos);

            if (projectile->isActive())
                continue;

            const auto target = projectile->getTarget();
            auto caster = projectileState.getCaster();
            assert(target != caster);

            if (caster.isEmpty())
                caster = target;

            // Try to get a Ptr to the bow that was used. It might no longer exist.
            MWWorld::ManualRef projectileRef(*MWBase::Environment::get().getESMStore(), projectileState.mIdArrow);
            MWWorld::Ptr bow = projectileRef.getPtr();
            if (!caster.isEmpty() && projectileState.mIdArrow != projectileState.mBowId)
            {
                MWWorld::InventoryStore& inv = caster.getClass().getInventoryStore(caster);
                MWWorld::ContainerStoreIterator invIt = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
                if (invIt != inv.end() && invIt->getCellRef().getRefId() == projectileState.mBowId)
                    bow = *invIt;
            }

            const auto hitPosition = Misc::Convert::toOsg(projectile->getHitPosition());

            if (projectile->getHitWater())
                mRendering->emitWaterRipple(hitPosition);

            const MWWorld::Ptr projectilePtr = projectileRef.getPtr();
            const bool hitActor = !target.isEmpty() && target.getClass().isActor();

            // A simulated item that was hit gets knocked, according to the momentum of the projectile.
            if (!target.isEmpty() && !hitActor && target.getClass().isItem(target))
            {
                const float projectileMass = std::max(projectilePtr.getClass().getWeight(projectilePtr), 0.1f);
                const float targetMass = std::clamp(target.getClass().getWeight(target), 0.2f, 50.f);
                osg::Vec3f direction = projectileState.mVelocity;
                direction.normalize();
                mPhysics->strikeObject(target, projectileState.mVelocity * (projectileMass / targetMass),
                    hitPosition, hitPosition - direction * 100.f);
            }

            const bool hitBody = MWMechanics::projectileHit(caster, target, bow, projectilePtr, hitPosition,
                projectileState.mAttackStrength, projectileState.mAttackWindUp);

            // A hit sticks in the body (not the player's: it would clutter the first person view).
            if (hitBody && target != MWMechanics::getPlayer())
            {
                if (MWRender::Animation* animation = MWBase::Environment::get().getWorld()->getAnimation(target))
                {
                    osg::Vec3f direction = projectileState.mVelocity;
                    direction.normalize();
                    animation->attachStuckProjectile(
                        projectilePtr.getClass().getCorrectedModel(projectilePtr), hitPosition, direction);
                }
            }

            // Missed shots aren't lost: the arrow, bolt or thrown weapon lands where it hit and can be picked up.
            // Enchanted ones are spent, their enchantment having gone off.
            if (!hitActor && projectilePtr.getClass().getEnchantment(projectilePtr).empty())
            {
                // Fixed things made of something soft enough (wood, plants, earth) catch it. It glances off stone
                // and metal, and off loose items, doors and water.
                bool stick = false;
                osg::Vec3f stickPoint = hitPosition;
                const bool hitTerrain = target.isEmpty() && !projectile->getHitWater();
                const bool hitStatic
                    = !target.isEmpty() && !target.getClass().isItem(target) && !target.getClass().isDoor();
                if (hitTerrain || hitStatic)
                {
                    osg::Vec3f direction = projectileState.mVelocity;
                    direction.normalize();
                    // The hit was on the collision shape, which can stand off from what is seen (often a little
                    // bigger). Look along the flight for the visible surface, a little either side of it.
                    const MWRender::RenderingManager::SurfaceResult surface
                        = mRendering->castRayForSurface(hitPosition - direction * 20.f, hitPosition + direction * 60.f);
                    const VFS::Path::Normalized model
                        = hitStatic ? target.getClass().getCorrectedModel(target) : VFS::Path::Normalized();
                    // Bare ground takes arrows unless it looks rocky; unrecognized objects deflect them.
                    stick = isSoftSurface(surface.mTexture, model.value(), hitTerrain || surface.mTerrain);
                    if (surface.mHit)
                        stickPoint = surface.mPosition;
                }
                placeMissedProjectile(projectileState, stick ? stickPoint : osg::Vec3f(hitPosition),
                    Misc::Convert::toOsg(projectile->getHitNormal()), stick);
            }

            projectileState.mToDelete = true;
        }

        for (auto& magicBoltState : mMagicBolts)
        {
            if (magicBoltState.mToDelete)
                continue;

            auto* projectile = mPhysics->getProjectile(magicBoltState.mProjectileId);

            const auto pos = projectile->getSimulationPosition();
            magicBoltState.mNode->setPosition(pos);
            for (const auto& sound : magicBoltState.mSounds)
                sound->setPosition(pos);

            const Ptr caster = magicBoltState.getCaster();

            const MWBase::World& world = *MWBase::Environment::get().getWorld();
            const bool active = projectile->isActive();
            if (active && !world.isUnderwater(caster.getCell(), pos))
                continue;

            const Ptr target = !active ? projectile->getTarget() : Ptr();

            assert(target != caster);

            auto hitPos = !active ? Misc::Convert::makeOsgVec3f(projectile->getHitPosition()) : pos;
            auto hitNormal = Misc::Convert::makeOsgVec3f(projectile->getHitNormal());

            if (projectile->getHitWater())
                mRendering->emitWaterRipple(hitPos);

            if (active)
            {
                hitNormal = projectile->velocity();
                hitNormal.normalize();
            }
            // A spell bolt striking a simulated item knocks it back (area effects are handled by the explosion).
            else if (!target.isEmpty() && !target.getClass().isActor() && target.getClass().isItem(target))
                mPhysics->strikeObject(target, hitNormal * -300.f, hitPos, hitPos + hitNormal * 100.f);
            MWBase::Environment::get().getLuaManager()->magicProjectileHit(
                magicBoltState.mSpellId, caster, magicBoltState.mItem, target, hitPos, hitNormal);

            magicBoltState.mToDelete = true;
        }

        for (auto& projectileState : mProjectiles)
        {
            if (projectileState.mToDelete)
                cleanupProjectile(projectileState);
        }

        for (auto& magicBoltState : mMagicBolts)
        {
            if (magicBoltState.mToDelete)
                cleanupMagicBolt(magicBoltState);
        }
        mProjectiles.erase(std::remove_if(mProjectiles.begin(), mProjectiles.end(),
                               [](const State& state) { return state.mToDelete; }),
            mProjectiles.end());
        mMagicBolts.erase(
            std::remove_if(mMagicBolts.begin(), mMagicBolts.end(), [](const State& state) { return state.mToDelete; }),
            mMagicBolts.end());
    }

    void ProjectileManager::placeMissedProjectile(
        const ProjectileState& state, const osg::Vec3f& hitPosition, const osg::Vec3f& hitNormal, bool stick)
    {
        MWBase::World* world = MWBase::Environment::get().getWorld();
        MWWorld::CellStore* cell = world->getPlayerPtr().getCell();
        if (cell == nullptr)
            return;

        osg::Vec3f direction = state.mVelocity;
        direction.normalize();
        // Backed out of whatever it hit, so it doesn't start inside it (a stuck one is driven in afterwards).
        const osg::Vec3f position = hitPosition - direction * 10.f;
        if (cell->isExterior())
            cell = &MWBase::Environment::get().getWorldModel()->getExterior(
                ESM::positionToExteriorCellLocation(position.x(), position.y(), cell->getCell()->getWorldSpace()));

        ESM::Position pos;
        pos.pos[0] = position.x();
        pos.pos[1] = position.y();
        pos.pos[2] = position.z();
        const osg::Vec3f rotation = MWPhysics::quatToEsmRotation(lookAt(state.mVelocity));
        pos.rot[0] = rotation.x();
        pos.rot[1] = rotation.y();
        pos.rot[2] = rotation.z();

        MWWorld::ManualRef ref(*MWBase::Environment::get().getESMStore(), state.mIdArrow, 1);
        const MWWorld::Ptr placed = world->placeObject(ref.getPtr(), cell, pos);
        if (stick)
            mPhysics->stickObject(placed, hitPosition);
        else
        {
            // Glance off what it hit, losing most of its speed.
            const osg::Vec3f reflected = state.mVelocity - hitNormal * (2.f * (state.mVelocity * hitNormal));
            mPhysics->deflectObject(placed, hitPosition, reflected * 0.25f);
        }
    }

    void ProjectileManager::cleanupProjectile(ProjectileManager::ProjectileState& state)
    {
        mParent->removeChild(state.mNode);
        mPhysics->removeProjectile(state.mProjectileId);
        state.mToDelete = true;
    }

    void ProjectileManager::cleanupMagicBolt(ProjectileManager::MagicBoltState& state)
    {
        mParent->removeChild(state.mNode);
        mPhysics->removeProjectile(state.mProjectileId);
        state.mToDelete = true;
        for (size_t soundIter = 0; soundIter != state.mSounds.size(); soundIter++)
        {
            MWBase::Environment::get().getSoundManager()->stopSound(state.mSounds.at(soundIter));
        }
    }

    void ProjectileManager::clear()
    {
        for (auto& mProjectile : mProjectiles)
            cleanupProjectile(mProjectile);
        mProjectiles.clear();

        for (auto& mMagicBolt : mMagicBolts)
            cleanupMagicBolt(mMagicBolt);
        mMagicBolts.clear();
    }

    void ProjectileManager::write(ESM::ESMWriter& writer, Loading::Listener& progress) const
    {
        for (const ProjectileState& projectile : mProjectiles)
        {
            writer.startRecord(ESM::REC_PROJ);

            ESM::ProjectileState state;
            state.mId = projectile.mIdArrow;
            state.mPosition = ESM::Vector3(osg::Vec3f(projectile.mNode->getPosition()));
            state.mOrientation = ESM::Quaternion(osg::Quat(projectile.mNode->getAttitude()));
            state.mCaster = projectile.mCaster;

            state.mBowId = projectile.mBowId;
            state.mVelocity = projectile.mVelocity;
            state.mAttackStrength = projectile.mAttackStrength;
            state.mAttackWindUp = projectile.mAttackWindUp;

            state.save(writer);

            writer.endRecord(ESM::REC_PROJ);
        }

        for (const MagicBoltState& bolt : mMagicBolts)
        {
            writer.startRecord(ESM::REC_MPRJ);

            ESM::MagicBoltState state;
            state.mId = bolt.mIdMagic.at(0);
            state.mPosition = ESM::Vector3(osg::Vec3f(bolt.mNode->getPosition()));
            state.mOrientation = ESM::Quaternion(osg::Quat(bolt.mNode->getAttitude()));
            state.mCaster = bolt.mCaster;
            state.mItem = bolt.mItem;
            state.mSpellId = bolt.mSpellId;
            state.mSpeed = bolt.mSpeed;

            state.save(writer);

            writer.endRecord(ESM::REC_MPRJ);
        }
    }

    bool ProjectileManager::readRecord(ESM::ESMReader& reader, uint32_t type)
    {
        if (type == ESM::REC_PROJ)
        {
            ESM::ProjectileState esm;
            esm.load(reader);

            ProjectileState state;
            state.mCaster = esm.mCaster;
            state.mBowId = esm.mBowId;
            state.mVelocity = esm.mVelocity;
            state.mIdArrow = esm.mId;
            state.mAttackStrength = esm.mAttackStrength;
            state.mAttackWindUp = esm.mAttackWindUp;
            state.mToDelete = false;

            VFS::Path::Normalized model;
            try
            {
                MWWorld::ManualRef ref(*MWBase::Environment::get().getESMStore(), esm.mId);
                MWWorld::Ptr ptr = ref.getPtr();
                model = ptr.getClass().getCorrectedModel(ptr);

                state.mProjectileId
                    = mPhysics->addProjectile(state.getCaster(), osg::Vec3f(esm.mPosition), model, false);
            }
            catch (const std::exception& e)
            {
                Log(Debug::Warning) << "Failed to add projectile for " << esm.mId
                                    << " while reading projectile record: " << e.what();
                return true;
            }

            createModel(state, model, osg::Vec3f(esm.mPosition), osg::Quat(esm.mOrientation), false, false,
                osg::Vec4(0, 0, 0, 0));

            mProjectiles.push_back(std::move(state));
            return true;
        }
        if (type == ESM::REC_MPRJ)
        {
            ESM::MagicBoltState esm;
            esm.load(reader);

            MagicBoltState state;
            state.mIdMagic.push_back(esm.mId);
            state.mSpellId = esm.mSpellId;
            state.mCaster = esm.mCaster;
            state.mToDelete = false;
            state.mItem = esm.mItem;
            VFS::Path::NormalizedView texture;

            try
            {
                state.mEffects = getMagicBoltData(
                    state.mIdMagic, state.mSoundIds, state.mSpeed, texture, state.mSourceName, state.mSpellId);
            }
            catch (const std::exception& e)
            {
                Log(Debug::Warning) << "Failed to recreate magic projectile for " << esm.mId << " and spell "
                                    << state.mSpellId << " while reading projectile record: " << e.what();
                return true;
            }

            state.mSpeed = esm.mSpeed; // speed is derived from non-projectile effects as well as
                                       // projectile effects, so we can't calculate it from the save
                                       // file's effect list, which is already trimmed of non-projectile
                                       // effects. We need to use the stored value.

            VFS::Path::Normalized model;
            try
            {
                MWWorld::ManualRef ref(*MWBase::Environment::get().getESMStore(), state.mIdMagic.at(0));
                MWWorld::Ptr ptr = ref.getPtr();
                model = ptr.getClass().getCorrectedModel(ptr);
            }
            catch (const std::exception& e)
            {
                Log(Debug::Warning) << "Failed to get model for " << state.mIdMagic.at(0)
                                    << " while reading projectile record: " << e.what();
                return true;
            }

            osg::Vec4 lightDiffuseColor = getMagicBoltLightDiffuseColor(state.mEffects);
            createModel(state, model, osg::Vec3f(esm.mPosition), osg::Quat(esm.mOrientation), true, true,
                lightDiffuseColor, texture);
            state.mProjectileId = mPhysics->addProjectile(state.getCaster(), osg::Vec3f(esm.mPosition), model, true);

            MWBase::SoundManager* sndMgr = MWBase::Environment::get().getSoundManager();
            for (const auto& soundid : state.mSoundIds)
            {
                MWBase::Sound* sound = sndMgr->playSound3D(
                    esm.mPosition, soundid, 1.0f, 1.0f, MWSound::Type::Sfx, MWSound::PlayMode::Loop);
                if (sound)
                    state.mSounds.push_back(sound);
            }

            mMagicBolts.push_back(std::move(state));
            return true;
        }

        return false;
    }

    size_t ProjectileManager::countSavedGameRecords() const
    {
        return mMagicBolts.size() + mProjectiles.size();
    }

    void ProjectileManager::saveLoaded(const ESM::ESMReader& reader)
    {
        // Can't do this in readRecord because the vectors might get reallocated as they grow
        if (reader.mActorIdConverter)
        {
            for (ProjectileState& projectile : mProjectiles)
                reader.mActorIdConverter->convert(projectile.mCaster, projectile.mCaster.mIndex);
            for (MagicBoltState& bolt : mMagicBolts)
                reader.mActorIdConverter->convert(bolt.mCaster, bolt.mCaster.mIndex);
        }
    }

    MWWorld::Ptr ProjectileManager::State::getCaster()
    {
        if (!mCasterHandle.isEmpty())
            return mCasterHandle;

        return MWBase::Environment::get().getWorldModel()->getPtr(mCaster);
    }

}

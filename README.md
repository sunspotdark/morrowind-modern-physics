Morrowind - Modern Physics (Unofficial OpenMW Fork)
===================================================

Hi, sunspotdark here. Below the line is a detailed AI generated summary of the actual changes and such, but I felt compelled to add a personal note here at the top written with my actual human hands. 

This is an unofficial fork of the OpenMW engine that adds Oblivion/Skyrim style physics to Morrowind. You can pick up items, spin them around and throw them. Or tenderly place them on a table or whatever. You do you. All enemies and NPCs have ragdoll physics and you can drag the bodies around. Stuff floats in the water. Arrows stick in the environment. Etc. Etc. Etc. 

I am not a programmer. At all. This MF is vibecoded to hell and back, specifically with Claude Opus 5.5. Frankly, I didn't think this was even going to work when I started poking at it, but guess that shows what I know. Chances are, there's a lot of broken stuff if you get deeper into the game, but everything around Seyda Neen seems to be working as intended so I figured why not share it. I may update this from time to time based on feedback or my own personal playthrough, but currently it seems to work well enough to toss out into the world.

I am led to believe by Claude that this probably works with old OpenMW saves, but new saves made within this fork will not be backwards compatible with OG OpenMW due to some funkiness with how the ragdoll physics are calculated, or some such. I have no idea if the slop machine is lying or not about this (see previous comment re: this being a vibecoded monstrosity made by a dribbling moron) so use at your own risk. I expect this probably works with most OpenMW mods that don't mess with the physics systems, but I cannot guarantee anything. The slop machine assures me that the ragdoll mechanics should apply to enemies added by mods, and there should be a failsafe if they don't. But, again, I have no idea if this is true--I cannot stress enough that I am exactly the sort of contemptible idiot who uses a $20 a month AI subscription to give themself delusions of game dev grandeur. But to my eyes, this works pretty much exactly as described. Crazy times we live in.

With respect to support expectations, this is a hobby project. I have a list of additional features I am in the process of testing out and adding (sound for physics interactions, throw distance scaling with strength, etc.), and I intend to keep it up to date with new OpenMW releases as possible (and, ultimately, deprecate it if they eventually get around to implementing the ragdoll physics, etc. stretch goals that duplicate the features of this fork), but you know how it can be with hobby projects. 

I have no affiliation with the OpenMW team or Bethesda, but I am, of course, deeply indebted to them.

Enjoy(?)

---------------------------------------------------------------------------------------------------------------

An **unofficial** fork of [OpenMW](https://openmw.org) that adds engine-level physics: clutter you can knock
over, pick up and throw, projectiles that stick where they land, and ragdoll corpses you can drag around.

> **This is not an official OpenMW release.** It is not made, reviewed or supported by the OpenMW team, and it
> is not affiliated with or endorsed by OpenMW, Bethesda Softworks or ZeniMax Media. Please **do not report
> problems with this fork to OpenMW's bug tracker, forums or Discord** — report them here instead.

You need to own *The Elder Scrolls III: Morrowind* to play. This fork, like OpenMW, contains no game data.

* Fork version: 1.0
* Based on: OpenMW 0.52.0 (development version, upstream commit `628b6c2b72`, 2026-10-03)
* Modified: October 2026, by sunspotdark (see [What this fork changes](#what-this-fork-changes) and the git
  history for the full list of changes)
* License: GPLv3, the same as OpenMW (see [LICENSE](LICENSE))

What this fork changes
----------------------

**Items are physical objects**
* Small items (clutter, weapons, ingredients, books...) are simulated rigid bodies: they fall, tumble, stack,
  roll off shelves and get shoved when you walk into them.
* Furniture gets detailed collision, so things rest *on* tables and *in* shelves instead of on their bounding
  boxes.
* Items float or sink in water by density (bottles bob, metal sinks), with ripples and splashes.

**Carrying and throwing**
* Hold **Activate** on an item to pick it up; a quick tap still takes it as usual.
* Press **Activate** again to put it down, or **Attack** to throw it. Thrown items can hit people (which counts
  as an attack, as it would be for any other hit).
* Hold **Rotate Held Item** (default **C**) and move the mouse to turn what you're holding.

**Combat knocks things about**
* Arrows, bolts, thrown weapons and spell projectiles knock items over; area spells push things away.
* Melee swings knock items in their path when they land.
* Missed arrows and bolts no longer vanish: they stick into soft things (wood, earth, flesh...) and bounce off
  hard ones (stone, metal), and can be picked up again. Arrows also stick visibly in the people they hit.

**Ragdolls**
* People and creatures go limp when they die, thrown by the killing blow, instead of playing a death animation.
  Creatures get a body fitted to their own model; creatures built from separate rigid pieces (scribs, shalks,
  Dwemer spheres...) fall apart.
* Hold **Activate** on a corpse to drag it by the limb you're looking at; a quick tap still loots it.
* Corpses are saved where and how they lie.
* People's bodies float in water.
* A ragdoll that goes wrong (for example with an unusual creature model from a mod) is replaced with the normal
  death pose.

Known issues and limitations
----------------------------

* **Saved games:** a save containing ragdolled corpses will most likely **not load in official OpenMW** (or
  other builds). Saves from official OpenMW load fine in this fork. Keep that in mind before switching back.
* Windows only for now (other platforms may build from source but are untested).
* Large furniture (beds, crates, tables) isn't simulated; it only has the detailed collision described above.
* Mods that change creature models are untested; see the ragdoll fail-safe above.
* Performance with very large numbers of simulated items in view has not been tested extensively.

Installing
----------

You need Windows 10 or 11 (64-bit) and an installed copy of Morrowind (GOG, Steam or disc).

1. Download the zip from the [Releases](../../releases) page and extract it to a folder of your choice (not
   inside `Program Files`, and not into an existing OpenMW folder).
2. Run `openmw-launcher.exe`.
   * If Windows SmartScreen warns about an unrecognised app, click **More info → Run anyway**. The build isn't
     code-signed.
   * If you get an error about a missing `MSVCP140.dll` or `VCRUNTIME140.dll`, install the
     [Microsoft Visual C++ Redistributable (x64)](https://aka.ms/vs/17/release/vc_redist.x64.exe) and try again.
3. On first run (if you've never used OpenMW before) the launcher offers to run the installation wizard. Accept, choose
   **Existing installation** and point it at your Morrowind folder (the one containing `Morrowind.esm`, usually
   `Data Files`). It will import your settings and content files.
4. Click **Play**.

**If you already use official OpenMW:** this fork uses the same settings and saves folder
(`Documents\My Games\OpenMW`), so it will pick up your existing setup, mods and saves straight away. Because
saves with ragdolled corpses won't load in official OpenMW (see [Known issues](#known-issues-and-limitations)),
**back up your `saves` folder first** if you plan to go back and forth.

To uninstall, delete the folder you extracted. Your settings and saves stay in `Documents\My Games\OpenMW`.

The data path, command line options and everything else work as they do in OpenMW (see
[About OpenMW](#about-openmw) below).

How this was made (AI disclosure)
---------------------------------

This fork was made with heavy use of AI. Almost all of the code was written by Claude (an AI model made by
Anthropic), working under my direction: I decided what to build and how it should behave, and tested each change
in game before it was kept. I am not a C++ developer, and the code has **not** been reviewed line by line by an
experienced OpenMW or engine developer — it has been checked by playing, not by expert review. Expect rough
edges, and please judge it accordingly. Commits written this way are marked with a `Co-Authored-By: Claude`
line in the git history.

Credits
-------

* All of OpenMW — the engine this fork is built on — is the work of the OpenMW team and contributors (see
  [AUTHORS.md](AUTHORS.md)). This fork only adds to it.
* The Windows build bundles third-party libraries (OpenSceneGraph, Bullet, Qt, FFmpeg, SDL2, OpenAL Soft, Boost,
  MyGUI, LuaJIT, ICU and others), each under its own license; their license texts are included in the release
  download.

Source code
-----------

The complete source for every release is in this repository; each release is tagged, and the downloadable build
is made from exactly that tag.

---

About OpenMW
============

*The rest of this file is OpenMW's own README, kept as it is. Its links point to the official OpenMW project,
which does not support this fork.*

OpenMW is an open-source open-world RPG game engine that supports playing Morrowind by Bethesda Softworks. You need to own the game for OpenMW to play Morrowind.

OpenMW also comes with OpenMW-CS, a replacement for Bethesda's Construction Set.

* Version: 0.52.0
* License: GPLv3 (see [LICENSE](https://gitlab.com/OpenMW/openmw/-/raw/master/LICENSE) for more information)
* Website: https://www.openmw.org
* IRC: #openmw on irc.libera.chat
* Discord: https://discord.gg/bWuqq2e


Font Licenses:
* DejaVuLGCSansMono.ttf: custom (see [files/data/fonts/DejaVuFontLicense.txt](https://gitlab.com/OpenMW/openmw/-/raw/master/files/data/fonts/DejaVuFontLicense.txt) for more information)
* DemonicLetters.ttf: SIL Open Font License (see [files/data/fonts/DemonicLettersFontLicense.txt](https://gitlab.com/OpenMW/openmw/-/raw/master/files/data/fonts/DemonicLettersFontLicense.txt) for more information)
* MysticCards.ttf: SIL Open Font License (see [files/data/fonts/MysticCardsFontLicense.txt](https://gitlab.com/OpenMW/openmw/-/raw/master/files/data/fonts/MysticCardsFontLicense.txt) for more information)

Current Status
--------------

The main quests in Morrowind, Tribunal and Bloodmoon are all completable. Some issues with side quests are to be expected (but rare). Check the [bug tracker](https://gitlab.com/OpenMW/openmw/-/issues/?milestone_title=openmw-1.0) for a list of issues we need to resolve before the "1.0" release. Even before the "1.0" release, however, OpenMW boasts some new [features](https://wiki.openmw.org/index.php?title=Features), such as improved graphics and user interfaces.

Pre-existing modifications created for the original Morrowind engine can be hit-and-miss. The OpenMW script compiler performs more thorough error-checking than Morrowind does, meaning that a mod created for Morrowind may not necessarily run in OpenMW. Some mods also rely on quirky behaviour or engine bugs in order to work. We are considering such compatibility issues on a case-by-case basis - in some cases adding a workaround to OpenMW may be feasible, in other cases fixing the mod will be the only option. If you know of any mods that work or don't work, feel free to add them to the [Mod status](https://wiki.openmw.org/index.php?title=Mod_status) wiki page.

Getting Started
---------------

* [Official forums](https://forum.openmw.org/)
* [Installation instructions](https://openmw.readthedocs.io/en/latest/manuals/installation/index.html)
* [Build from source](https://wiki.openmw.org/index.php?title=Development_Environment_Setup)
* [Testing the game](https://wiki.openmw.org/index.php?title=Testing)
* [How to contribute](https://wiki.openmw.org/index.php?title=Contribution_Wanted)
* [Report a bug](https://gitlab.com/OpenMW/openmw/issues) - read the [guidelines](https://wiki.openmw.org/index.php?title=Bug_Reporting_Guidelines) before submitting your first bug!
* [Known issues](https://gitlab.com/OpenMW/openmw/issues?label_name%5B%5D=Bug)

The data path
-------------

The data path tells OpenMW where to find your Morrowind files. If you run the launcher, OpenMW should be able to pick up the location of these files on its own, if both Morrowind and OpenMW are installed properly (installing Morrowind under WINE is considered a proper install).

Command line options
--------------------

    Syntax: openmw <options>
    Allowed options:
      --config arg                          additional config directories
      --replace arg                         settings where the values from the
                                            current source should replace those
                                            from lower-priority sources instead of
                                            being appended
      --user-data arg                       set user data directory (used for
                                            saves, screenshots, etc)
      --resources arg (=resources)          set resources directory
      --help                                print help message
      --version                             print version information and quit
      --data arg (=data)                    set data directories (later directories
                                            have higher priority)
      --data-local arg                      set local data directory (highest
                                            priority)
      --fallback-archive arg (=fallback-archive)
                                            set fallback BSA archives (later
                                            archives have higher priority)
      --start arg                           set initial cell
      --content arg                         content file(s): esm/esp, or
                                            omwgame/omwaddon/omwscripts
      --groundcover arg                     groundcover content file(s): esm/esp,
                                            or omwgame/omwaddon
      --no-sound [=arg(=1)] (=0)            disable all sounds
      --script-all [=arg(=1)] (=0)          compile all scripts (excluding dialogue
                                            scripts) at startup
      --script-all-dialogue [=arg(=1)] (=0) compile all dialogue scripts at startup
      --script-console [=arg(=1)] (=0)      enable console-only script
                                            functionality
      --script-run arg                      select a file containing a list of
                                            console commands that is executed on
                                            startup
      --script-warn [=arg(=1)] (=1)         handling of warnings when compiling
                                            scripts
                                            0 - ignore warnings
                                            1 - show warnings but consider script as
                                            correctly compiled anyway
                                            2 - treat warnings as errors
      --load-savegame arg                   load a save game file on game startup
                                            (specify an absolute filename or a
                                            filename relative to the current
                                            working directory)
      --skip-menu [=arg(=1)] (=0)           skip main menu on game startup
      --new-game [=arg(=1)] (=0)            run new game sequence (ignored if
                                            skip-menu=0)
      --encoding arg (=win1252)             Character encoding used in OpenMW game
                                            messages:

                                            win1250 - Central and Eastern European
                                            such as Polish, Czech, Slovak,
                                            Hungarian, Slovene, Bosnian, Croatian,
                                            Serbian (Latin script), Romanian and
                                            Albanian languages

                                            win1251 - Cyrillic alphabet such as
                                            Russian, Bulgarian, Serbian Cyrillic
                                            and other languages

                                            win1252 - Western European (Latin)
                                            alphabet, used by default
      --fallback arg                        fallback values
      --no-grab [=arg(=1)] (=0)             Don't grab mouse cursor
      --export-fonts [=arg(=1)] (=0)        Export Morrowind .fnt fonts to PNG
                                            image and XML file in current directory
      --activate-dist arg (=-1)             activation distance override
      --random-seed arg (=<impl defined>)   seed value for random number generator

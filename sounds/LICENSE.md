# Sound file licensing and origins

The **source code** of this project is licensed under the
[GNU General Public License v3](../LICENSE). **The audio files are not.** They are media,
not code: the GPL covers the program that plays them, not the noise it makes, and nothing in
this directory is licensed under the GPL by us.

This file does not hand out a licence either. It records what is known about each file, so
that nobody has to guess.

Two kinds of audio file live here:

| files                                  | what                                        | terms                                                                                |
| -------------------------------------- | ------------------------------------------- | ------------------------------------------------------------------------------------ |
| `<language>/*.mp3`, `<language>_*.mp3` | spoken announcements, generated from a text | the TTS provider's licence, see [point 1](#1-generated-announcements-soundslanguage) |
| `zz_*.mp3`                             | effects and short music intros              | mixed, see [point 2](#2-effects-and-music-zz_mp3)                                    |

## 1. Generated announcements (`sounds/<language>/`)

These are not recordings of a person but renders of a text by a TTS engine. The engines we
support draw on a stock catalogue of voices - no clone of a real person, so nothing here
impersonates anybody - and the output is licensed by the provider:

| engine                              | licence of the generated audio                                                                                         |
| ----------------------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| [ElevenLabs](https://elevenlabs.io) | The rights in the output are yours; commercial use needs a paid plan ([terms](https://elevenlabs.io/terms-of-use)).    |
| [Cartesia](https://www.cartesia.ai) | Non-commercial by default, commercial only where the booked plan allows it ([terms](https://cartesia.ai/legal/terms)). |

So an announcement carries the licence of the account that rendered it. **We claim no
copyright in these files** and do not put the GPL on them; the renders shipped in the images
come from the maintainers' account, not from yours.

The files are not committed (`.gitignore`: `sounds/*/`) because they are generated in CI and
on the robot - but they still end up in the image, which is why they are covered here.

## 2. Effects and music (`zz_*.mp3`)

| file                        | where it came from                                                                                                   |
| --------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| `zz_pink-panther.mp3`       | free ringtone from [klingeltonekostenlos.de](https://klingeltonekostenlos.de/pink-panther-1/); the theme is © MGM/UA |
| `zz_highway-to-hell.mp3`    | fan cover by [Philbert Audio](https://soundcloud.com/simonpvo/acdc-highway-to-hell); the song is © J. Albert & Son   |
| `zz_coming-home.mp3`        | shortened public demo ([Spotify](https://open.spotify.com/intl-de/track/1yQiYwG2N8WiWHlhj3CX3T))                     |
| `zz_coming-home-remix.mp3`  | free remix, [ReverbNation](https://www.reverbnation.com/n3xg3nmusik/song/9300198-diddy-dirty-money-coming-home-zouk) |
| `zz_sweet-home-alabama.mp3` | uploaded to the [Internet Archive](https://archive.org/details/lynyrd.-skynyrd.2/)                                   |
| `zz_rain-singing.mp3`       | [The Bangerz remix](https://soundcloud.com/exit33/singing-in-the-rain-the-bangerz-remix)                             |
| `zz_rain-supertramp.mp3`    | uploaded to the [Internet Archive](https://archive.org/details/supertramp-music/)                                    |
| `zz_rain-garbage.mp3`       | [cover by Giulio Peruch](https://soundcloud.com/giulio-peruch/garbage-only-happy-when-it-rains-cover)                |

These are fan recordings, remixes and archive uploads of music that is somebody else's work -
a "free" download link describes the download, not the rights. Each is a short excerpt (15 to
30 seconds, used as an intro, never a whole track), which explains the length but is no
licence. None of it is covered by the GPL.

The remaining effects, like `zz_ping`, `zz_burp`, `zz_cow`, `zz_sheep`, `zz_fart-wet`,
`zz_fart-vegan`, `zz_minion-fire-alarm`, `zz_minion-fart`, are short samples and
synthesised blips, several of them made for this project; the two Minion-ish ones are
imitations made by a fan, not audio from a film. They have no author to speak of and no
licence of their own: treat them as licence-free.

## 3. If you hold the rights to one of these files

[Open an issue](https://github.com/ClemensElflein/open_mower_ros/issues) or write to the
maintainer and the file is removed - no discussion needed, no proof required. A proper source
or a licence for one of the rows above is just as welcome.

## 4. Disclaimer

No legal advice, no warranty: a good-faith record of what we know, and of the providers'
terms as of 2026-11-07. Where it matters, read the linked original.

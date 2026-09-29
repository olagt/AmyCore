# Contributing

Thanks for your interest in AmyCore.

## Contributor License Agreement

AmyCore is dual-licensed (AGPL-3.0 and a commercial license, see [LICENSE-COMMERCIAL.md](LICENSE-COMMERCIAL.md)). To
keep that possible, every contribution needs a signed [Contributor License Agreement](CLA.md) before it can be
merged. The CLA lets the project license your contribution under both licenses; you keep the copyright to your work.

When you open your first pull request, the CLA Assistant bot asks you to sign by commenting on it. You only sign once.

## Safety

AmyCore moves a real industrial arm. Changes to frame code, the command path, enable/homing sequences or the
ESP32 firmware must be tested with care: keep the E-stop in reach, start with low speeds and stiffness, and describe
in your pull request how and on what hardware the change was tested. Read [docs/JRCP_PROTOCOL.md](docs/JRCP_PROTOCOL.md)
before changing frame code.

## Style

Match the surrounding code: naming, comment density and idiom. Keep new dependencies permissively licensed
(MIT, BSD, Apache-2.0, zlib, MPL-2.0); copyleft dependencies can't be used because of the commercial license.

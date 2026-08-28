# Third-party notices

The BSD 3-Clause License in the repository root applies to the original
project code by Yuwei Zhang, unless a source file or directory contains a
different notice.

## Raspberry Pi Pico SDK

The `pico-sdk-master/` directory is the Raspberry Pi Pico SDK and remains
under its upstream license. Its license text and copyright notices are kept
with that SDK. The root `pico_sdk_import.cmake` file is adapted from the Pico
SDK and retains the Raspberry Pi Trading Ltd. BSD 3-Clause notice in the file
itself.

When redistributing this repository with the SDK included, retain the SDK's
original license files and notices. The project license does not replace or
relicense the SDK.

## OV5640-related material

The OV5640-related files may contain material adapted from or carrying notices
from STMicroelectronics or other upstream sources. In particular,
`header/ov5640.h` retains an STMicroelectronics copyright and license notice.
Those files are governed by their applicable upstream notices, which take
precedence over the project license. Retain those notices when redistributing
and verify the upstream terms before publishing modified or binary versions.

## Other dependencies

Pico SDK dependencies and toolchain components may have their own licenses.
Consult the corresponding upstream distributions for their complete license
terms.

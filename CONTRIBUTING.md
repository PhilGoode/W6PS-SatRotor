# Contributing to W6PS SatRotor

This project is not publicly launched yet, but this document is being added now
so the repo has a sane contribution shape when it does move to GitHub.

## Current Reality

- this is an active bench-build project
- hardware and firmware are evolving together
- repo docs are being prepared with future public release in mind

## Contribution Intent

For now, contributions should assume:

- correctness beats cleverness
- hardware assumptions must be written down
- protocol behavior should be documented, not guessed
- changes that affect motion/control logic should stay conservative

## Design Principles

- Keep the `Nucleo H743ZI / H753ZI` as the motion/control authority
- Keep the `Raspberry Pi` focused on UI and front-panel behavior
- Keep external protocol compatibility separate from internal bridge/control messages
- Prefer original implementations over copy/paste from GPL projects

## Documentation Expectations

If a change affects behavior, also update the docs that explain that behavior.

That usually means one or more of:

- [README.md](README.md)
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/PROJECT_STATUS.md](docs/PROJECT_STATUS.md)
- [firmware/README.md](firmware/README.md)
- [firmware/docs/PROTOCOL_IMPLEMENTATION_CHECKLIST.md](firmware/docs/PROTOCOL_IMPLEMENTATION_CHECKLIST.md)

## Code Style Expectations

- keep embedded logic simple and explicit
- avoid hidden state transitions
- keep protocol parsing readable
- preserve bench-friendly logging where it helps debugging
- keep UI-related parsing and state handling separate from rendering where practical

## Licensing / Reuse Boundary

This project may study outside open-source controllers for behavior, protocol
coverage, and interoperability lessons.

Do not:

- copy GPL code into this repository
- port GPL source with minor edits
- drop in third-party code without checking license impact first

See [LICENSE-STATUS.md](LICENSE-STATUS.md).

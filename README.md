# pnp-z-center
An extention to the opulu pick and place machine of the Z-Axis

The idee is, to have almost the standard Z cariage, which will HOME to the center position, so that both nozles are in a save position.
Furthere more, both axes can be controlled independently (only one after the other). For that, a second virtual axis is introduced called W.

It is also possible to attach a blTouch device, which will trigger and the current altitude can be read back.

This is work in progress, but got to a point, which may be interesting to others

State:
- mechanical changes: a print out exists, which can mount the needed light barrier
- a rust FW running on an STM32F103 mounted on an old mobo rev3 board from mai 2021 is available too

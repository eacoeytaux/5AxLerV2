# 5AxLerV2
5AxLer converted to be a sub-part of CuraEngine

This software is made to generate G-code for any STL file that fits the specifications for
a 5 axis 3D printer that uses a fixed nozzle.

This repo is split into two part: Slicer and Viewer

## Slicer

Slicer is the backend slicing component that generates G-code for the printer without any UI.
It is a fork on the CuraEngine with an addition "5axis" folder which adds the 5 axis features
to the slicing process.

## Viewer

Viewer is totally independent of Slicer.  Given a set of sliced STLs the Viewer allows the user
to visualize the separated components using openGL.  It does no operations on the STL and is a
tool using for testing and to verify the STL was slicing correctly.

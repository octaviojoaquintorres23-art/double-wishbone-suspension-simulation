# double-wishbone-suspension-simulation
This is a python program which attempts to simulate the core geometry of a double wishbone suspension setup in 3D space.
This is done by controlling the degrees of freedom of a few members, and then using geomtrical constraints to figure out the rest.
As such, this is not a true kinematic analysis of a suspension setup, as it does not simulate a dynamic (nor transient) scenario.
Still, it can calculate some neat points such as the Ackerman turning center, instance centers, contact patch centers, and roll center
in real time with varying bump & steer combinations.

One specialty of this program is that it can handle very odd setups, such as when the control arm chassis mounting points are not parallel to
each other.

Please note, this program does these calculations in 3D space. So, nothing is idealized to a 2D drawing, and the roll center (and other
commonly known points) come out to be lines rather than points.

<img width="1342" height="843" alt="image" src="https://github.com/user-attachments/assets/59fa1c4d-bcab-41c7-a4d4-8fcd04bf3bbe" />

Currently configured controls:

[a] - move steering rack left

[d] - move steering rack right

Arrow Keys - move camera view around

[i] - zoom in

[k] - zoom out

# To-do:

- Fix the study feature
- Fix wheel definition
- Add easier way of defining geometry
- Possibly implement instant study-to-plot creation

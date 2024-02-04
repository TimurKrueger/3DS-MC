# 3DS-MC Interactive ARAP
As-Rigid-As-Possible surface deformation (ARAP) is a method used in computer graphics and geometric processing for deforming shapes while preserving rigidity. It aims to maintain the local rigidity of a deforming object, ensuring that nearby points move as little as possible relative to each other.
You can find further information about this project in the report(Add link to report).

## Setup and Control
Following Steps need to be done in order for you to run the project:
- **OpenGL** needs to be installed on your computer
- libigl will be automatically installed via CMake ( **[libigl](https://libigl.github.io/tutorial/)** )

Once you configured and generated with CMake you can run the project.
Press on Load Mesh and search for a .ply/.off file. (Unfortunately you need to type in the name and press enter)


Once the Mesh is loaded there are several Controls:
- "1": Enter/exit the state in which you can select the faces that will not move by clicking on the face. While the mouse button is held you can simply move over multiple faces.
**Video von select Faces**
- "3": Enter the ARAP state in which you can drag a vertex around by clicking and dragging it with the mouse. 
**Video von arap**

## Contributors
- Köse, Ömer
- Adam, Natalie
- Krüger, Timur
- Peis, Kilian
# Differential Growth Model

A model simulating differential growth.

## Requirements
### To run the model
- Rust: [https://www.rust-lang.org/](https://www.rust-lang.org/)

### To generate the geometry and load it
- Rhino & Grasshopper [https://www.rhino3d.com/](https://www.rhino3d.com/)


## Run
- Open the GeometryExport.gh file in Grasshopper.
- Reference the colliders and origin Polylines. 
- Save the resulting text file as "geometry.pp".
- Run the growth model: `cargo run --release`
- Press `p` once it is stabilized.
- Point the data path in Grasshopper to the generated `out.pp` file. A polyline of the generated curve should appear. 
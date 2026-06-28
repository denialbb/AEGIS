import krpc
import os
import sys

def main():
    print("Connecting to kRPC...")
    try:
        # AEGIS uses 172.22.80.1 and ports 50000/50001
        conn = krpc.connect(name="AEGIS_Obj_Exporter", address="172.22.80.1", rpc_port=50000, stream_port=50001)
    except Exception as e:
        print(f"Failed to connect to KSP: {e}")
        print("Ensure KSP is running on the launchpad with the kRPC server active.")
        sys.exit(1)
        
    vessel = conn.space_center.active_vessel
    ref_frame = vessel.reference_frame
    
    print(f"Connected! Exporting bounding boxes for vessel: {vessel.name}")
    
    vertices = []
    faces = []
    
    vertex_offset = 1
    
    # Iterate through every part to get its bounding box relative to the CoM
    for part in vessel.parts.all:
        try:
            bbox = part.bounding_box(ref_frame)
        except Exception:
            continue
            
        xmin, ymin, zmin = bbox[0]
        xmax, ymax, zmax = bbox[1]
        
        # We map kRPC coordinates (X=Right, Y=Forward, Z=Down) 
        # to standard Blender Space (X=Right, Y=Forward, Z=Up)
        # by flipping the Z axis: Z_obj = -Z_ksp
        corners_ksp = [
            (xmin, ymin, zmin), # 0
            (xmax, ymin, zmin), # 1
            (xmax, ymax, zmin), # 2
            (xmin, ymax, zmin), # 3
            (xmin, ymin, zmax), # 4
            (xmax, ymin, zmax), # 5
            (xmax, ymax, zmax), # 6
            (xmin, ymax, zmax), # 7
        ]
        
        for cx, cy, cz in corners_ksp:
            # Map Z -> -Z
            vertices.append((cx, cy, -cz))
            
        v = vertex_offset
        
        # Define the 6 faces using counter-clockwise winding (for outward normals)
        faces.append((v+3, v+2, v+1, v+0)) # Bottom (Zmin in kRPC, Zmax in Blender)
        faces.append((v+4, v+5, v+6, v+7)) # Top
        faces.append((v+2, v+3, v+7, v+6)) # Front
        faces.append((v+0, v+1, v+5, v+4)) # Back
        faces.append((v+0, v+4, v+7, v+3)) # Left
        faces.append((v+1, v+2, v+6, v+5)) # Right
        
        vertex_offset += 8

    os.makedirs("assets", exist_ok=True)
    out_path = "assets/ksp_vessel_hitbox.obj"
    
    with open(out_path, "w") as f:
        f.write("# AEGIS KSP Bounding Box Exporter\n")
        f.write(f"# Vessel: {vessel.name}\n")
        f.write(f"o {vessel.name.replace(' ', '_')}\n")
        
        for vx, vy, vz in vertices:
            f.write(f"v {vx:.4f} {vy:.4f} {vz:.4f}\n")
            
        for face in faces:
            f.write(f"f {face[0]} {face[1]} {face[2]} {face[3]}\n")
            
    print(f"Success! Exported {len(vessel.parts.all)} parts to '{out_path}'.")
    print("You can now import this file into Blender and improve upon it.")

if __name__ == '__main__':
    main()

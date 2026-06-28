# KSP Mesh Extraction & Integration Workflow

## Step 1: Install Mu Importer

1. Download `io_object_mu.zip` [^1].
2. In Blender: **Edit > Preferences > Add-ons**.
3. Click **Install...**, select the zip, and enable **Import-Export: Mu model format**.

## Step 2: Extract KSP Parts

1. Navigate to the KSP GameData directory [^2].
2. Locate the `.mu` (mesh) and `.dds`/`.png` (texture) files for your lander's components.
3. Copy them to a local working folder.

## Step 3: Assemble in Blender

1. Clear the default Blender scene.
2. Go to **File > Import > KSP Mu (.mu)** and import your copied parts.
3. Assemble the parts to match your `.craft` design using translation (`G`) and snapping.
4. Move all parts so the expected Center of Mass (CoM) is exactly at `(0,0,0)` [^3].

## Step 4: Export Model

1. Select all parts (`A`).
2. Go to **File > Export > glTF 2.0 (.glb/.gltf)** [^4].
3. In export settings: Check **Selected Objects**.
4. In export settings: Ensure **+Y Up** is checked under Transform.
5. Export as `aegis_lander.glb` directly into your project's `assets/` folder.

## Step 5: Update Python Script

1. Load the model during script initialization: `model = pr.load_model("assets/aegis_lander.glb")`
2. Draw the model in your render loop using `pr.draw_model_ex` [^5].
3. Unload the model before closing the window: `pr.unload_model(model)`

## Legal Notice & The "Clean Room" Alternative

> [!WARNING]
> KSP meshes are the copyrighted intellectual property of Take-Two Interactive. **Do not** upload extracted `.glb`, `.obj`, or `.mu` files to public repositories (like GitHub) or distribute them. Modifying the mesh geometry or removing textures still constitutes a derivative work and is not protected by fair use.

### The 100% Legal Exporter Script

If you wish to share a 3D representation of your vessel online, you can mathematically generate a primitive representation based on live KSP telemetry. We have included `scripts/export_hitbox_obj.py` in the repository for this exact purpose:

1. Launch KSP and load your vessel onto the pad with kRPC running.
2. Run `wsl -d Arch .venv/bin/python scripts/export_hitbox_obj.py`.
3. The script iterates through `vessel.parts.all` and queries their exact physical bounding boxes.
4. It mathematically writes a 1-to-1 scale wireframe `.obj` file to `assets/ksp_vessel_hitbox.obj` [^6].

## You can load this generated `.obj` directly into Raylib using `pr.draw_model_wires_ex()` or import it into Blender as a legally safe template to build your own "lookalike" mesh from scratch.

[^1]: The zip file is located at the [taniwha/io_object_mu GitHub repository](https://github.com/taniwha/io_object_mu). Do not extract the zip before installing.

[^2]: For stock parts on WSL, this is typically `/mnt/c/Games/KSP - minimal install/GameData/Squad/Parts/`.

[^3]: Our `PhysicsState` tracks the CoM. Placing the Blender origin at the CoM ensures Raylib renders the model perfectly aligned with the physics engine. Ensure the vessel points UP along the Z-axis.

[^4]: glTF is recommended because it bundles meshes, materials, and textures into a single efficient file natively supported by Raylib.

[^5]: You will need to convert the quaternion to an axis-angle representation, and map the NED axis `(N, E, D)` to Raylib's Y-up axis `(N, -D, E)` for the `axis` parameter.

[^6]: Because this script generates its own primitive geometry mathematically (rather than copying the proprietary mesh data), it circumvents copyright restrictions and produces an asset that is 100% legally yours.

bl_info = {
    "name": "PLY to Mesh Converter",
    "blender": (4, 4, 0),
    "category": "Import-Export",
    "version": (1, 0, 0),
    "author": "Abishek",
    "description": "Import point clouds and convert to mesh using Open3D"
}
import bpy
import open3d as o3d
import numpy as np
import os
from bpy.props import (StringProperty, FloatProperty, IntProperty, 
                      EnumProperty, PointerProperty)
from bpy.types import Operator, Panel, PropertyGroup
from bpy_extras.io_utils import ImportHelper

# Property group for persistent settings
class PointCloudSettings(PropertyGroup):
    file_path: StringProperty(
        name="File Path",
        description="Path to point cloud file",
        subtype='FILE_PATH'
    )
    
    reconstruction_method: EnumProperty(
        name="Method",
        description="Surface reconstruction algorithm",
        items=[
            ('POISSON', "Poisson", "Poisson reconstruction for organic shapes"),
            ('BALL_PIVOTING', "Ball Pivoting", "Ball pivoting for mechanical parts")
        ],
        default='POISSON'
    )
    
    poisson_depth: IntProperty(
        name="Depth",
        description="Tree depth for Poisson reconstruction (higher = more detail)",
        default=9,
        min=1,
        max=15
    )
    
    normal_radius: FloatProperty(
        name="Normal Radius",
        description="Search radius for normal estimation",
        default=0.1,
        min=0.01
    )
    
    max_nn: IntProperty(
        name="Max Neighbors",
        description="Maximum nearest neighbors for normal estimation",
        default=30,
        min=1
    )
    
    bp_radii: StringProperty(
        name="Ball Radii",
        description="Comma-separated radii for ball pivoting (e.g., 0.05,0.1,0.2)",
        default="0.05, 0.1, 0.2"
    )

# File browser operator
class OpenFileBrowser(Operator, ImportHelper):
    bl_idname = "pc.open_file_browser"
    bl_label = "Select Point Cloud"
    bl_options = {'REGISTER', 'UNDO'}

    filter_glob: StringProperty(
        default="*.ply;*.pcd;*.xyz;*.xyzn;*.xyzrgb;*.pts;*.txt",
        options={'HIDDEN'}
    )

    def execute(self, context):
        context.scene.pc_settings.file_path = self.filepath
        return {'FINISHED'}

# Main processing operator
class PLYToMeshOperator(Operator):
    bl_idname = "object.ply_to_mesh"
    bl_label = "Convert Point Cloud to Mesh"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        pc_settings = context.scene.pc_settings
        file_path = bpy.path.abspath(pc_settings.file_path)  # Convert to absolute path

        # Check if the file exists
        if not os.path.exists(file_path):
            self.report({'ERROR'}, f"File not found: {file_path}")
            return {'CANCELLED'}

        try:
            self.report({'INFO'}, f"Loading file: {file_path}")
            pcd = o3d.io.read_point_cloud(file_path)

            if pcd.is_empty() or len(pcd.points) == 0:
                self.report({'ERROR'}, f"Loaded point cloud is empty or unreadable: {file_path}")
                return {'CANCELLED'}

            # Normal estimation
            pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=pc_settings.normal_radius, 
                max_nn=pc_settings.max_nn
            ))

            # Surface reconstruction
            if pc_settings.reconstruction_method == 'BALL_PIVOTING':
                try:
                    radii = [float(r.strip()) for r in pc_settings.bp_radii.split(',')]
                except ValueError:
                    self.report({'ERROR'}, "Invalid ball radii format. Use comma-separated floats.")
                    return {'CANCELLED'}

                mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
                    pcd, o3d.utility.DoubleVector(radii))
            else:
                mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
                    pcd, depth=pc_settings.poisson_depth)

            # Validate mesh
            if len(mesh.vertices) == 0 or len(mesh.triangles) == 0:
                self.report({'ERROR'}, "Reconstruction failed: mesh is empty")
                return {'CANCELLED'}

            # Convert to Blender mesh
            vertices = np.asarray(mesh.vertices)
            faces = np.asarray(mesh.triangles)

            bl_mesh = bpy.data.meshes.new("ConvertedMesh")
            bl_mesh.from_pydata(vertices.tolist(), [], faces.tolist())

            obj = bpy.data.objects.new("PointCloudMesh", bl_mesh)
            context.collection.objects.link(obj)

            self.report({'INFO'}, "Mesh created successfully")
            return {'FINISHED'}

        except Exception as e:
            self.report({'ERROR'}, f"Exception occurred: {str(e)}")
            return {'CANCELLED'}

# Panel UI
class PC_PT_MainPanel(Panel):
    bl_label = "Point Cloud to Mesh"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Tool'

    def draw(self, context):
        layout = self.layout
        pc_settings = context.scene.pc_settings

        # File selection
        col = layout.column()
        col.prop(pc_settings, "file_path")
        col.operator("pc.open_file_browser", text="Browse", icon='FILEBROWSER')

        # Reconstruction method
        layout.separator()
        layout.prop(pc_settings, "reconstruction_method", text="Method")

        # Method-specific parameters
        if pc_settings.reconstruction_method == 'POISSON':
            col = layout.column()
            col.prop(pc_settings, "poisson_depth")
            col.prop(pc_settings, "normal_radius")
            col.prop(pc_settings, "max_nn")
        else:
            col = layout.column()
            col.prop(pc_settings, "bp_radii")
            col.prop(pc_settings, "normal_radius")

        # Convert button
        layout.separator()
        layout.operator("object.ply_to_mesh", text="Convert", icon='IMPORT')

# Register/unregister
classes = [
    PointCloudSettings,
    OpenFileBrowser,
    PLYToMeshOperator,
    PC_PT_MainPanel
]

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.pc_settings = PointerProperty(type=PointCloudSettings)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.pc_settings

if __name__ == "__main__":
    register()
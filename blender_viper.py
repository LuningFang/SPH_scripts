import bpy
import math
import mathutils
import csv
import sys
time = []

# Total number of parts of the rover
num_body = 17   # num bodies per rover
num_rovers = 3  # number of rovers
num_links = num_rovers - 1  # links connecting rovers
num_plate = 1
num_rocks = 5   # number of rocks
total_num_bodies = num_body * num_rovers + num_plate + num_links + num_rocks

# lookup table that maps body name with obj mesh name
# duplicated rover or rock bodies are not listed
map = {
    "body00" : "viper_chassis_trans",
    "body01" : "viper_wheel",
    "body02" : "viper_L_up_sus",
    "body03" : "viper_L_bt_sus",
    "body04" : "viper_L_steer",
    "body05" : "viper_wheel",
    "body06" : "viper_R_up_sus",
    "body07" : "viper_R_bt_sus",
    "body08" : "viper_R_steer",
    "body09" : "viper_wheel",
    "body10" : "viper_L_up_sus",
    "body11" : "viper_L_bt_sus",
    "body12" : "viper_L_steer",
    "body13" : "viper_wheel",
    "body14" : "viper_R_up_sus",
    "body15" : "viper_R_bt_sus",
    "body16" : "viper_R_steer",
}

# Resolution of the output image
res_x = 1920
res_y = 900

# res_x = 800
# res_y = 450


# Specify the directory of csv, obj, image
data_sim = "./"
file_obj = "./obj_for_render/"
image_dir = "./animation/"




# find the position that the camera points at
def point_at(obj, target, roll=0):
    """
    Rotate obj to look at target
    :arg obj: the object to be rotated. Usually the camera
    :arg target: the location (3-tuple or Vector) to be looked at
    :arg roll: The angle of rotation about the axis from obj to target in radians.
    """
    if not isinstance(target, mathutils.Vector):
        target = mathutils.Vector(target)
    loc = obj.location
    # direction points from the object to the target
    direction = target - loc

    quat = direction.to_track_quat('-Z', 'Y')

    # /usr/share/blender/scripts/addons/add_advanced_objects_menu/arrange_on_curve.py
    quat = quat.to_matrix().to_4x4()
    rollMatrix = mathutils.Matrix.Rotation(roll, 4, 'Z')

    # remember the current location, since assigning to obj.matrix_world changes it
    loc = loc.to_tuple()
    #obj.matrix_world = quat * rollMatrix
    # in blender 2.8 and above @ is used to multiply matrices
    # using * still works but results in unexpected behaviour!
    obj.matrix_world = quat @ rollMatrix
    obj.location = loc

# find the position and rotation of the camera
def rot_pos(center, radius, angle=0):
    """
    return (x,y,z) points on a circle centered at (center) with a radius of (radius) with an angle of (angle)
    """
    # convert to radian
    angle_pi = angle/180*math.pi
    # generate point on circle
    return (center[0]+radius*math.sin(angle_pi),center[1]+radius*math.cos(angle_pi),center[2])

# load position and quaternion of each body at different time frame
rot = []
dist = []
for i in range(total_num_bodies):
    with open(data_sim + "/rover/body_pos_rot_vel" + str(i) + ".csv", 'r') as file:
        rot_temp = []
        dist_temp = []

        reader = csv.reader(file)
        next(reader)
        for row in reader:
            dist_temp.append((float(row[1]), float(row[2]), float(row[3])))
            rot_temp.append((float(row[4]), float(row[5]), float(row[6]), float(row[7])))
        dist.append(dist_temp)
        rot.append(rot_temp)


# frame_id
i = int(sys.argv[4])

#===========================================
#======== check if the png file exits or not
#===========================================
# image_path = image_dir + str(i) + ".png"
# file_exists = os.path.exists(image_path)
# if file_exists:
#     sys.exit()

# generate blender scene
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.objects.keys()

#===========================================
#===================== load rover obj file
#===========================================
for nr in range(3):
    for n in range(num_body):

        body_name = "body" + str(n).zfill(2)
        # corresponding obj mesh name
        obj_name = map[body_name]

        bpy.ops.import_scene.obj(filepath = file_obj + obj_name + ".obj")
        # Get the last imported object
        imported_object = bpy.context.selected_objects[0]

        # Change the name of the imported object to reflect viper id and body id
        viper_body_name = "viper" + str(nr) + "_" + body_name
        imported_object.name = viper_body_name

        body_id = n + nr * num_body

        bpy.data.objects[viper_body_name].location = dist[body_id][i]

        # bpy.data.objects[obj_name_spe].scale = (0.1,0.1,0.1)
        bpy.data.objects[viper_body_name].rotation_mode = 'QUATERNION'
        bpy.data.objects[viper_body_name].rotation_quaternion = rot[body_id][i]

bpy.context.view_layer.update()

################
## load terrain ##
################
obj_name = "soilmesh_" + str(i+1).zfill(4)
file_loc = "terrain/" + obj_name + ".obj"
imported_object = bpy.ops.import_scene.obj(filepath = file_loc)
obj_object = bpy.context.object


# Create new material for terrain
material_name = "my_material"
new_material = bpy.data.materials.new(name=material_name)
new_material.diffuse_color = (0.2, 0.2, 0.2, 1)  # Example color
new_material.specular_color = (0.5, 0.5, 0.5)  # Example color
new_material.specular_intensity = 96 / 1000.0

if bpy.data.objects[obj_name].data.materials:
   bpy.data.objects[obj_name].data.materials[0] = new_material
else:
    bpy.data.objects[obj_name].data.materials.append(new_material)


bpy.ops.transform.rotate(value=(math.pi * 0.5), orient_axis='X')  # value = Angle
bpy.context.view_layer.update()



################
## load link ##
################
# obj_name = "link"
# file_loc = file_obj + obj_name + ".obj"
# imported_object = bpy.ops.import_scene.obj(filepath = file_loc)
# obj_object = bpy.context.object
# body_id = 51
# bpy.data.objects[obj_name].location = dist[body_id][i]
# bpy.data.objects[obj_name].rotation_mode = 'QUATERNION'
# bpy.data.objects[obj_name].rotation_quaternion = rot[body_id][i]
# bpy.context.view_layer.update()


# ################
# ## load blade ##
# ################
# obj_name = "link"
# file_loc = file_obj + obj_name + ".obj"
# imported_object = bpy.ops.import_scene.obj(filepath = file_loc)
# obj_object = bpy.context.object
# body_id = 52
# bpy.data.objects[obj_name].location = dist[body_id][i]
# bpy.data.objects[obj_name].rotation_mode = 'QUATERNION'
# bpy.data.objects[obj_name].rotation_quaternion = rot[body_id][i]
# bpy.context.view_layer.update()



################
## load blade ##
################
obj_name = "blade"
file_loc = file_obj + obj_name + ".obj"
imported_object = bpy.ops.import_scene.obj(filepath = file_loc)
obj_object = bpy.context.object
body_id = 53
bpy.data.objects[obj_name].location = dist[body_id][i]
bpy.data.objects[obj_name].rotation_mode = 'QUATERNION'
bpy.data.objects[obj_name].rotation_quaternion = rot[body_id][i]
bpy.context.view_layer.update()


################
## load rocks ##
################
rock_scale = 0.2
for rock_id in range(num_rocks):
    obj_name = "rock"
    obj_name_spe = "rock"

    if rock_id > 0:
        obj_name_spe = "rock." + str(rock_id).zfill(3)

    file_loc = file_obj + obj_name + ".obj"
    imported_object = bpy.ops.import_scene.obj(filepath = file_loc)
    obj_object = bpy.context.object
    body_id = 54 + rock_id
    bpy.data.objects[obj_name_spe].location = dist[body_id][i]

    bpy.data.objects[obj_name_spe].scale = (rock_scale, rock_scale, rock_scale)

    bpy.data.objects[obj_name_spe].rotation_mode = 'QUATERNION'
    bpy.data.objects[obj_name_spe].rotation_quaternion = rot[body_id][i]
bpy.context.view_layer.update()

#===========================================
#========================= Rendering setting
#===========================================
bpy.ops.transform.rotate(value=(-math.pi * 0.5), orient_axis='X')  # value = Angle
bpy.ops.mesh.primitive_plane_add(size=200.0, calc_uvs=True, enter_editmode=False,
                                    align='WORLD', location=(0.0, 0.0, -0.58))

#======== create a camera and settings
bpy.ops.object.camera_add(enter_editmode=False, align='WORLD', scale=(5.0, 5.0, 5.0))
scene.camera = bpy.context.object
# Set up rotational camera
cam = bpy.data.objects["Camera"]
# add rotational angle by 1
ini_rad = 135
cur_rad = ini_rad + i * 0.005
camera_radius = 15

# cam.location = rot_pos((dis[0][i][0],dis[0][i][1],dis[0][i][2]+4),10,cur_rad)
cam.location = rot_pos((3,0,4), camera_radius, cur_rad)
point_at(cam, (dist[0][i][0],dist[0][i][1],dist[0][i][2]+0.5), roll=math.radians(0))
# point_at(cam, (2,0,0.5), roll=math.radians(0))

scene.cycles.device = 'GPU'

prefs = bpy.context.preferences
cprefs = prefs.addons['cycles'].preferences

# Attempt to set GPU device types if available
for compute_device_type in ('CUDA', 'OPENCL', 'NONE'):
    try:
        cprefs.compute_device_type = compute_device_type
        break
    except TypeError:
        pass

# Enable all CPU and GPU devices
cprefs.get_devices()
for device in cprefs.devices:
    device.use = True

#======== create light datablock, set attributes
light_data = bpy.data.lights.new(name="light_2.80", type='POINT')
light_data.energy = 12000
# create new object with our light datablock
light_object = bpy.data.objects.new(name="light_2.80", object_data=light_data)
# link light object
bpy.context.collection.objects.link(light_object)
# make it active
bpy.context.view_layer.objects.active = light_object
# change location
light_object.location = (10, 10, 15)

#======== create another light datablock, set attributes
light_data1 = bpy.data.lights.new(name="light_top", type='POINT')
light_data1.energy = 1500
# create new object with our light datablock
light_object1 = bpy.data.objects.new(name="light_top", object_data=light_data1)
# link light object
bpy.context.collection.objects.link(light_object1)
# make it active
bpy.context.view_layer.objects.active = light_object1
# change location
light_object1.location = ( dist[0][i][0],  dist[0][i][1], 15 )

bpy.context.scene.render.engine = 'CYCLES'
bpy.context.scene.cycles.device = 'GPU'
bpy.context.scene.render.resolution_percentage = 100
bpy.context.scene.cycles.samples = 256
bpy.context.scene.render.resolution_x = res_x
bpy.context.scene.render.resolution_y = res_y
bpy.context.scene.render.filepath = image_dir + str(i).zfill(4) + ".png"
#bpy.context.scene.render.image_settings.compression = 50
bpy.context.scene.render.image_settings.color_mode = 'RGBA'
bpy.context.scene.render.image_settings.file_format = 'PNG'
bpy.ops.render.render(write_still=True)

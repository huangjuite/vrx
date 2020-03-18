import lxml.etree
import lxml.builder
import random

uri_lib = {
    "red_totem": "model://red_totem",
    "green_totem": "model://green_totem",
    "yellow_totem": "model://yellow_totem",
    "black_totem": "model://black_totem",
    "blue_totem": "model://blue_totem",
    "ball3": "model://polyform_a3",
    "ball5": "model://polyform_a5",
    "ball7": "model://polyform_a7",
    # "dock": "model://dock_block_4x4_dynamic"
}

wall_num = 40
wall_width = 10
block_num = 10
block_distance = 10
position_noise = 3


def include_model(model_name, model_pose, model_uri):
    E = lxml.builder.ElementMaker()
    include = E.include
    uri = E.uri
    name = E.name
    pose = E.pose

    part = include(
        uri(model_uri),
        name(model_name),
        pose("%.2f %.2f %.2f %.2f %.2f %.2f" % (
            model_pose[0], model_pose[1], model_pose[2], model_pose[3], model_pose[4], model_pose[5]))
    )
    outupt = lxml.etree.tostring(part, pretty_print=True)
    print outupt
    return outupt


f = open("world_part.txt", "w+")
pos_f = open("block_position.csv", "w+")
pos_f.write("object,x,y\n")
# # create right wall
# for i in range(wall_num):
#     part = include_model("right_dock%d" %
#                          i, [i*2, -wall_width/2., 0, 0, 0, 0], uri_lib['dock'])
#     f.write(part)

# # create left wall
# for i in range(wall_num):
#     part = include_model("left_dock%d" %
#                          i, [i*2, wall_width/2., 0, 0, 0, 0], uri_lib['dock'])
#     f.write(part)

for i in range(block_num):
    for k in range(block_num):
        choice = random.choice(list(uri_lib))
        x_noise = random.random()*position_noise
        y_noise = random.random()*position_noise
        x = i*block_distance+x_noise-block_distance*block_num/2.+block_distance/2.
        y = k*block_distance+y_noise-block_distance*block_num/2.+block_distance/2.
        part = include_model("block%d%d" % (i, k), [x, y,0, 0, 0, 0], uri_lib[choice])
        f.write(part)
        pos_f.write("%s,%f,%f\n"%(choice,x,y))

pos_f.close()
f.close()

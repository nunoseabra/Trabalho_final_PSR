#!/usr/bin/env python3

import json
import rospkg

# TODO: Change the dictionary from robutler_bringup to robutler_description (Change all the launch files and rospgk functions) 

def robot_dictionary():
    rob_bedroom_sections = {
        "bedroom_entrance": {"Coords": [-2.229286, -0.150375, -3.134459]},  # X, Y, Yaw
        "bed": {"Coords": [-4.276079, -0.220221, 2.279560]},
        "bedside_table_right": {"Coords": [-4.661233, 0.087952, 1.110907]},
        "bed_table": {"Coords": [-6.908179, -0.312084, 2.399119]},
        "bed_table_on_top": {"Coords": [-8.058504, 1.088103, 2.434028]},
        "bedside_table_left": {"Coords": [-7.769165, 1.606756, 1.580553]},
    }

    rob_gym_sections = {
        "gym_table": {"Coords": [-0.390597, 1.749288, 1.622612]},
        "gym_exercise_area": {"Coords": [1.622612, 2.222902, 0.471165]},
        
    }

    rob_living_room_sections = {
        "wide_view": {"Coords": [5.406128, -1.667074, -2.954676]},
        "coffee_table": {"Coords": [0.178091, 0.163724, -0.792975]},
        "sofa": {"Coords": [1.722356, 0.638375, -2.108056]},
        "cabinet": {"Coords": [-2.039584, -3.304789, -0.685617]},
        "shoe_rack": {"Coords": [3.416510, -2.624828, -1.399908]},
    }

    rob_dining_room_sections = {
        "living_room_view": {"Coords": [4.050766, -0.511902, 0.574028]},
        "top_dining_table": {"Coords": [5.314251, 0.305187, 0.573831]},
        "kitchen_view": {"Coords": [6.573831, -1.138432, 1.523631]},
    }

    rob_kitchen_sections = {
        "living_room_view": {"Coords": [4.416283, -2.000127, -0.160161]},
        "dining_room_view": {"Coords": [6.451540, -1.331393, -1.320177]},
        "stove": {"Coords": [6.931216, -3.201187, -0.090585]},
        "countertop": {"Coords": [7.468817, -3.048542, -1.323980]},
    }

    rob_random_room_sections = {
        "entrance": {"Coords": [-2.237149, -1.924746, -2.729596]},
        "center": {"Coords": [-6.020113, -3.479462, -2.729570]},
    }

    robutler_check_loc = {
        "bedroom": rob_bedroom_sections,
        "gym": rob_gym_sections,
        "living_room": rob_living_room_sections,
        "dining_room": rob_dining_room_sections,
        "kitchen": rob_kitchen_sections,
        "random_room": rob_random_room_sections,
    }

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("robutler_bringup_23_24")
    fullpath = pkg_path + "/dictionary/robutler_check_loc.txt"
    with open(fullpath, "w") as convert_file:
        convert_file.write(json.dumps(robutler_check_loc, indent=2))
    convert_file.close()


def object_dictionary():
    obj_bedroom_sections = {
        "bedside_table_right": {
            #                  X,        Y,         Z,   Roll,Pitch,Yaw
            "Coords": [-4.353799, 2.865052, 0.784565, 0, 0, 0.060633],
            "Size": "Small",
        },
        "bed": {
            "Coords": [-6.234550, 1.841569, 0.772517, -0.007876, 0.003780, 0.35],
            "Size": "Big",
        },
        "bedside_table_left": {
            "Coords": [-7.566754, 2.889087, 0.758539, -0.000050, 0.000024, 0],
            "Size": "Small",
        },
        "bed_table": {
            "Coords": [-8.251611, 1.906945, 0.513816, 0, 0, 0.713857],
            "Size": "Small",
        },
        "bed_table_on_top": {
            "Coords": [-8.996136, 2.048792, 0.51, 0.000877, -0.003197, 0.71],
            "Size": "Small",
        },
        "in_the_open": {
            "Coords": [-7.466659, -0.044414, 0.066231, 0.000050, -0.000026, 1.031184],
            "Size": "Big",
        },
    }

    obj_gym_sections = {
        "gym_table": {
            "Coords": [-0.561677, 3.965602, 0.468291, -0.042383, 0.008629, 1.752721],
            "Size": "Small",
        },
    
        "gym_exercise_bench": {
            "Coords": [3.066207, 3.127018, 0.407947, 0.000018, -0.000046,-1.263798],
            "Size": "Big",
        },
    }

    obj_living_room_sections = {
        "sofa": {
            "Coords": [0.027281, -2.499502, 0.558637, -0.004788, 0.040572, 2.956630],
            "Size": "Big",
        },
        "coffee_table": {
            "Coords": [1.231942, -1.698951, 0.431986, 0.000020, -0.000032, 3.127639],
            "Size": "Small",
        },
        "cabinet": {
            "Coords": [1.189978, -5.084969, 0.558289, 0.000001, -0.000001, -2.123016],
            "Size": "Small",
        },
        "shoe_rack": {
            "Coords": [4.026049, -5.177916, 0.754706, 0, 0, 1.326251],
            "Size": "Small",
        },
        "in_the_open": {
            "Coords": [3.187537, -4.503206, 0.066227, 0, 0, 2.834155],
            "Size": "Big",
        },
    }

    obj_dining_room_sections = {
        "in_the_open_1": {
            "Coords": [5.320904, 2.482418, 0.066230, 0.000020, -0.000035, -0.336266],
            "Size": "Big",
        },
        "top_dining_table": {
            "Coords": [6.174503, 0.991941, 0.884683, 0.000041, -0.000018, -0.936365],
            "Size": "Small",
        },
        "in_the_open_2": {
            "Coords": [8.200308, 1.768009, 0.066232, -0.000053, -0.000025, -0.207331],
            "Size": "Big",
        },
    }

    obj_kitchen_sections = {
        "in_the_open_1": {
            "Coords": [8.210065, -4.171718, 0.066229, 0.000028, -0.000010, -2.147107],
            "Size": "Big",
        },
        "in_the_open_2": {
            "Coords": [8.069761, -1.299014, 0.066227, -0.000012, 0.000006, -0.980578],
            "Size": "Big",
        },
        "stove": {
            "Coords": [9.071861, -3.576553, 1.067766, 0.000018, -0.000050, -1.263798],
            "Size": "Small",
        },
        "countertop": {
            "Coords": [8.971446, -5.071446, 0.975436, -0.000026, 0.000017, 2.511640],
            "Size": "Small",
        },
    }

    obj_random_room_sections = {
        "in_the_open": {
            "Coords": [-5.441856, -4.032910, 0.066226, -0.000003, 0.000000, 1.656644],
            "Size": "Big",
        },
        "chair": {
            "Coords": [-8.296339, -4.469997, 0.467944, 0.064908, -0.003812, 2.404442],
            "Size": "Small",
        },
    }

    object_spawn_loc = {
        "bedroom": obj_bedroom_sections,
        "gym": obj_gym_sections,
        "living_room": obj_living_room_sections,
        "dining_room": obj_dining_room_sections,
        "kitchen": obj_kitchen_sections,
        "random_room": obj_random_room_sections,
    }

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("robutler_bringup_23_24")
    fullpath = pkg_path + "/dictionary/object_spawn_loc.txt"
    with open(fullpath, "w") as convert_file:
        convert_file.write(json.dumps(object_spawn_loc, indent=2))
    convert_file.close()


def main():
    robot_dictionary()
    object_dictionary()


if __name__ == "__main__":
    main()

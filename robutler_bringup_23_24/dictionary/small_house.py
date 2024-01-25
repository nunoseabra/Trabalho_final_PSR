#!/usr/bin/env python3

import json


def main():
    bedroom_sections = {
        "bedside_table_right": {"Coords": [-4.661233, 0.087952, 1.110907]},  # X, Y, Yaw
        "bed": {"Coords": [-4.276079, -0.220221, 2.279560]},
        "bedside_table_left": {"Coords": [-7.769165, 1.606756, 1.580553]},
        "bed_table": {"Coords": [-6.908179, -0.312084, 2.399119]},
        "bed_table_on_top": {"Coords": [-8.058504, 1.088103, 2.434028]},
        "bedroom_entrance": {"Coords": [-2.229286, -0.150375, -3.134459]},
    }

    gym_sections = {
        "gym_table": {"Coords": [-0.390597, 1.749288, 1.622612]},
        "gym_exercise_area": {"Coords": [1.622612, 2.222902, 0.471165]},
    }

    living_room_sections = {
        "sofa": {"Coords": [1.722356, 0.638375, -2.108056]},
        "coffee_table": {"Coords": [0.178091, 0.163724, -0.792975]},
        "cabinet": {"Coords": [-2.039584, -3.304789, -0.685617]},
        "shoe_rack": {"Coords": [3.416510, -2.624828, -1.399908]},
        "wide_view": {"Coords": [5.406128, -1.667074, -2.954676]},
    }

    dining_room_sections = {
        "living_room_view": {"Coords": [4.050766, -0.511902, 0.574028]},
        "top_dining_table": {"Coords": [5.314251, 0.305187, 0.573831]},
        "kitchen_view": {"Coords": [0.573831, -1.138432, 1.523631]},
    }

    kitchen_sections = {
        "living_room_view": {"Coords": [4.416283, -2.000127, -0.160161]},
        "dining_room_view": {"Coords": [6.451540, -1.331393, -1.320177]},
        "stove": {"Coords": [6.931216, -3.201187, -0.090585]},
        "countertop": {"Coords": [7.468817, -3.048542, -1.323980]},
    }

    random_room_sections = {
        "entrance": {"Coords": [-2.237149, -1.924746, -2.729596]},
        "center": {"Coords": [-6.020113, -3.479462, -2.729570]},
    }

    small_house = {
        "bedroom": bedroom_sections,
        "gym": gym_sections,
        "living_room": living_room_sections,
        "dining_room": dining_room_sections,
        "kitchen": kitchen_sections,
        "random_room": random_room_sections,
    }

    with open("small_house.txt", "w") as convert_file:
        convert_file.write(
            json.dumps(small_house, indent=2)
        )  # added indentation for better readability


if __name__ == "__main__":
    main()

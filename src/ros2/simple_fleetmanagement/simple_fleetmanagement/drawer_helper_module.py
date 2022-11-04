from typing import List


def get_list_of_drawer_ids(list_of_drawers) -> List[int]:
    list_of_drawer_ids = []
    for item in list_of_drawers:
        drawer_id = item["drawer_controller_id"]
        list_of_drawer_ids.append(drawer_id)
    return list_of_drawer_ids

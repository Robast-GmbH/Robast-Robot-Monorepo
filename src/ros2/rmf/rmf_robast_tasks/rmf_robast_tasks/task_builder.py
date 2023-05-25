def fill_phases( actions: list[str]):
    list_of_actions=[]
    for action in actions:
        body={}
        body["activity"]=action
        list_of_actions.append(body)    
    return list_of_actions
    
def fill_compose(actions : list[str], description_of_the_task:str):
    # Define task request description with phases
    description = {}  
    description["detail"] = description_of_the_task
    description["phases"] = fill_phases(actions)
    return description

def fill_go_to_place(place:str):
    activity = {}
    activity["category"] = "go_to_place"
    activity["description"] = place
    return activity

def perform_action(name:str, parameters:str ):
    activity = {}
    activity["category"] = "perform_action"
    custom_action = {}
    custom_action["category"] = name #"customer_interaction"
    custom_action["description"] = parameters
    activity["description"] = custom_action
    return activity
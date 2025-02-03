#!/usr/bin/env python3
import json 

def read_json_file(file_path, root): 

    data = None 
    try:
        # Opening JSON file
        with open(file_path, 'r') as f:
            # returns JSON object as a dictionary
            data = json.load(f)[root]
            
    except FileNotFoundError:
        print("Error: The file was not found.")
    except json.JSONDecodeError:
        print("Error: There was an error decoding the JSON data.")
    except KeyError:
        print("Error: The key %s was not found in the JSON data.", root)
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

    return data 

def json_root_keys_to_list(file_path, root): 
    data = read_json_file(file_path, root)
    return list(data.keys())

def name_to_id_dict(dict, root): 
    return {item['name']: item['id'] for item in dict.get(root).values()}

def id_to_name_dict(dict, root): 
    return {item['id']: item['name'] for item in dict.get(root).values()}

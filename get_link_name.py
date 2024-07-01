import requests
import json

def get_gazebo_links(model_name):
    url = f'http://localhost:11345/gazebo/model/{model_name}/link'
    
    try:
        response = requests.get(url)
        response.raise_for_status()  # Raise error for bad response status

        # Parse the JSON response
        links_data = response.json()
        
        # Extract link names
        link_names = [link['name'] for link in links_data['links']]
        
        return link_names
    
    except requests.exceptions.RequestException as e:
        print(f"Error fetching links for model '{model_name}': {e}")
        print(f"Response content: {response.content}")  # Print response content for debugging
        return []

# Example usage:
model_name = "iris_demo"
link_names = get_gazebo_links(model_name)
print(f"Links of model '{model_name}': {link_names}")

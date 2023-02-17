import requests
import json
from urllib.error import HTTPError


__all__ = [
    "MirAPI"
]


class MirAPI:
    def __init__(self, prefix, headers, api_yaml, timeout=10.0, debug=False):

        #HTTP connection
        self.prefix =  prefix
        self.debug = debug
        self.headers = headers
        self.timeout = timeout
        self.connected = False
        self.api_yaml = api_yaml
        print("in init class API", self.api_yaml)

        # Test connectivity
        try:
            response = requests.get(self.prefix + 'wifi/connections', headers=self.headers, timeout=self.timeout)
            self.connected = True
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
         
    def status_get(self):
        if not self.connected:
            return
        try:
            response = requests.get(self.prefix + f'status', headers=self.headers, timeout=self.timeout)
            print("========status_get=======")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def missions_get(self):
        if not self.connected:
            return
        try:
            response = requests.get(self.prefix + 'missions', headers = self.headers, timeout = 1.0)
            print("===========mission_get==========")
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
    
    def positions_get(self):
        if not self.connected:
            return
        try:
            response = requests.get(self.prefix + 'positions' , headers=self.headers, timeout=self.timeout)
            print("========positions_get=======")
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def mission_queue_post(self, mission_id):
        if not self.connected:
            return
        data = {"mission_id": mission_id}
        try:
            print("data", data)
            print("===============================")
            print("===============================")
            print("===============================")
            response = requests.post(self.prefix + 'mission_queue' , headers = self.headers, data=json.dumps(data), timeout = self.timeout)
            print("========mission_queue_post=======")
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other here error: {err}")

    def missions_mission_id_actions_post(self,mission_id,body):
        if not self.connected:
            return
        try:
            response = requests.post(self.prefix + 'missions/' +mission_id +'/actions' , headers = self.headers, data=json.dumps(body), timeout = self.timeout)
            print("========missions_mission_id_actions_post=======")
            if self.debug:
                print(f"Response: {response.json()}")
                print(response.status_code)
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def missions_post(self, mission):
        if not self.connected:
            return
        try:
            response = requests.post(self.prefix + 'missions' , headers = self.headers, data=mission, timeout = self.timeout)
            print("========missions_post=======")
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def positions_guid_get(self, guid):
        if not self.connected:
            return
        try:
            response = requests.get(self.prefix + 'positions/'+ guid, headers=self.headers, timeout=self.timeout)
            # print("========positions_guid_get=======")
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    def status_put(self, state_id):
        if not self.connected:
            return
        payload = json.dumps({
            "state_id": "020d1185-56a3-11ed-b824-94c6911e788e"
        })
        # data = {"state_id": state_id}
        try:
            response = requests.put(self.prefix + 'status/', headers = self.headers, data=payload, timeout=self.timeout)
            print("==========statusidddd=========")
            if self.debug:
                  print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
                
    def mission_queue_delete(self):
        if not self.connected:
            return
        try:
            response = requests.delete(self.prefix + 'missions' , headers = self.headers, timeout = self.timeout)
            print("==========mission_queue_delete=========")
            if self.debug:
                print(f"Response: {response.headers}")
            return True
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
            return False
        except Exception as err:
            print(f"Other error: {err}")
            return False

    def missions_guid_delete(self, guid):
        if not self.connected:
         return
        try:
            response = requests.delete(self.prefix + 'missions/' +guid , headers = self.headers, timeout = self.timeout)
            print("==========missions_guid_delete=========")
            if self.debug:
                print(f"Response: {response.headers}")
            return True
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
            return False
        except Exception as err:
            print(f"Other  error: {err}")
            return False
    

    ####################################
    ### CUSTOM API COMMANDS BY SAJID ###
    ####################################

    # USED TO CHANGE MAP IN ROBOT
    def map_put(self,level):
        if not self.connected:
            return
        
        payload = json.dumps({
                "map_id": self.api_yaml[level]["map_id"],
                "position": {
                    "orientation": self.api_yaml[level]["ori"],
                    "x": self.api_yaml[level]["x"],
                    "y": self.api_yaml[level]["y"]
                }
            })
        try:
            response = requests.request("PUT", self.prefix + 'status', headers = self.headers, data=payload)
            print("==========map_put=========")
            
            if True:
                  print("self.prefix",self.prefix)
                  print("headers", self.headers)
                  print("data", payload)
                  print("====change map======")
                  print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    # GET CURRENT MAP NAME OF ROBOT
    def map_print(self):
        if not self.connected:
            return
        try:
            response = requests.request("GET", self.prefix + 'status', headers=self.headers, timeout=self.timeout)
            print("===========map_print==========")
            response_dictionary = response.json()
            extracted_dictionary = response_dictionary['map_id']
            response = requests.request("GET", self.prefix + 'maps', headers=self.headers, timeout=self.timeout)
            for map in response.json():
                if map.get('guid') == extracted_dictionary:
                    map_name = map.get('name')
            if self.debug:
                print(f"Response: {response.json()}")
            print(map_name)
            return map_name
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")

    # POSTING SVY21 COORDINATES TO ODP FOR DIGITAL TWIN
    def odp_location(self,x,y,lvl):
        url = self.api_yaml['robot-location']

        print("====x",x)
        print("=====y",y)
        print("======lvl",lvl)

        data = json.dumps({'robotId':'ROBOT-5', 'level':str(lvl), 
                            'latitude':str(y), 'longitude':str(x)})
        
        headers = {
        'Content-Type': 'application/json'
        }

        print("====payload==",data)

        response = requests.request("POST", url, headers=headers, data=data)

        print(response.text)
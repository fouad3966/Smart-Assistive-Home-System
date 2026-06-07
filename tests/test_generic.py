import sys
sys.path.insert(0, '/home/boethius/autonomous_car/navigation/test')
from navigator import Navigator

def run_test():
    # Provide the stations file explicitly
    nav = Navigator('/home/boethius/autonomous_car/navigation/stations.json')
    
    # Overwrite show_map to not display plots
    import navigator
    navigator.show_map = lambda *args, **kwargs: print("[map] disabled for testing")
    
    # Also overwrite sleep and input so it goes faster without hanging
    import time
    time.sleep = lambda t: None
    import builtins
    builtins.input = lambda prompt="": print(f"[mock input] {prompt}")
    
    try:
        print(">>> ROUTING TO STATION 1")
        nav.Maps_to('station1')
        print(">>> ROUTING TO STATION 2")
        nav.Maps_to('station2')
        print(">>> ALL TESTS RAN")
    except Exception as e:
        print(f"Exception: {e}")

run_test()

import subprocess
import re

def main():
    result = subprocess.run(["sudo", "lshw", "-class", "network"], capture_output=True)
    result_str = result.stdout.decode("utf-8")
    interfaces = re.split("  \\*-network", result_str)

    interface_name = None

    for i in interfaces:
        # Use bus info to detect if it's usb
        if len(i) > 0 and re.search("(bus info: usb)", i):
            # Find interface name
            interface_matches = re.findall("logical name: (eth[0-9]+)", i)
            if len(interface_matches) > 0:
                interface_name = interface_matches[0]
                print("Interface name: " + interface_name)
    
    subprocess.run(["bash", "./lidar_setup.sh", interface_name, "non-interactive"])

if __name__ == "__main__":
    main()


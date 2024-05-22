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
            interface_match = re.search("logical name: (eth[0-9]+)", i)
            if interface_match:
                interface_name = interface_match.group(0)
                print("Interface name: " + interface_name)
    
    subprocess.run([f"sudo ip addr flush dev {interface_name}"])
    subprocess.run([f"ip addr show dev {interface_name}"])
    subprocess.run([f"sudo ip addr add 10.5.5.1/24 dev {interface_name}"])
    subprocess.run([f"sudo ip link set {interface_name} up"])
    subprocess.run([f"ip addr show dev {interface_name}"])

    subprocess.run(["sudo systemctl stop dnsmasq"])
    subprocess.run([f"sudo dnsmasq -C /dev/null -kd -F 10.5.5.96,10.5.5.96 -i {interface_name} --bind-dynamic"])
    subprocess.run(["ping -c1 os1-992005000098.local"])

if __name__ == "__main__":
    main()


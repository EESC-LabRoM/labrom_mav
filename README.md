# LabRoM MAV stack

**Summary**: This LabRoM stack contains packages that support micro aerial vehicles (MAVs) basic functionalities such as autonomous take-off, hovering, waypoint following, tracking and landing.

### **Important**: You are welcome to contribute.
Click on the figure below to see some flights that were performed using this stack:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [![](https://img.youtube.com/vi/v3DzcVIi7Ec/mqdefault.jpg)](https://www.youtube.com/watch?v=v3DzcVIi7Ec) &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; [![](https://img.youtube.com/vi/z2mk09uS8gQ/mqdefault.jpg)](https://www.youtube.com/watch?v=z2mk09uS8gQ)

### Notes:
* **labrom_mav_common**: Package that contains base data structure and functions shared by packages within this stack. 
* **labrom_mav_control**: Package that contains position controller.
* **labrom_mav_demos**: Package that contains launch files for running required nodes at once.
* **labrom_mav_interface**: Package that contains interfaces for different drivers such that messages are transmitted in the required format.
* **labrom_mav_manager**: Package that contains a basic high level entity that receives user commands (take-off, waypoints, etc) and supervises the vehicle behaviour throughout the flight.




.

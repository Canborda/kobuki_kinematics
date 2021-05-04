# RyCSV: Assignment 1
## Kobuki Kinematics

### Authors:

<table>
    <tr>
        <td>Camilo Andrés Borda Gil</td>
        <td>caabordagi@unal.edu.co</td>
    </tr>
    <tr>
        <td>Daniel Fernando Diaz Coy</td>
        <td>dafdiazco@unal.edu.co</td>
    </tr>
    <tr>
        <td>Andrés Felipe Rivera Torres</td>
        <td>afriverat@unal.edu.co</td>
    </tr>
</table>

### Description:

- For manual launch `$ roslaunch kobuki_kinematics test_simulation.launch`:
    1. Will run RViz and Gazebo environments and load the model.
    1. Run `$ rosrun kobuki_kinematics kobuki_teleop` to publish velocities modified by keys.
    1. Run `$ rosrun kobuki_kinematics kobuki_model` to subscribe, convert and publish the wheel velocities.

- For complete launch `$ roslaunch kobuki_kinematics kobuki_final.launch`.

---
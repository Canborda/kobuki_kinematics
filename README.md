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

1. For manual launch `$ roslaunch kobuki_kinematics test_simulation.launch`.
    - Will run RViz and Gazebo environments and load the model.
    - Then must run `$ rosrun kobuki_kinematics kobuki_teleop`
1. Then (on a new terminal) make `$ rosrun kobuki_kinematics kobuki_node.py` to open the <b>teleop keys</b>
1. To end just kill the nodes with `ctrl+C` or `$ rosnode kill /vel_publisher...`.

---
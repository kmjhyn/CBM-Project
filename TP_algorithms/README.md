# TP Algorithms
This code is an edited version of the [original github code](https://github.com/Lodz97/Multi-Agent_Pickup_and_Delivery) of the paper, [Robust Multi-Agent Pickup and Delivery with Delays](https://arxiv.org/abs/2303.17422).
It is edited by Ji Hyun Kim for CNM course project implementations.</p>
<p>The following readme is also an edited version including the algorithm structures for all three variations of TP algorithms (TP-replanning, K-TP, P-TP) used in the final project to find the optimal Multi-Agent Path Finding, Pickup and Delivery algorithm.
It also contains specific guidelines on how to re-run our experiments, with instructions on folders that contains the final outcome that is stated on the final report.
</p>

## File Overview
 <ul>
    <li> demo_JH.py: main file to run the code </li>
    <li> config.json: file where you can change the environment for each experiments </li>
    <li> Environments Folder: folder with all four environments,one extra environment (Environments/CBM_complex_big_moretask_JH.yaml) used for initial testing </li>
    <li> output.yaml: output yaml file where the results are automatically saved after running demo_JH file </li>
    <li> final_results Folder: folder with all the results in yaml format for each experiment and the plotted images of the results. </li>
    <li> requirements.txt: All the packages needed to run the code</li>

</ul>


## Structure
The structure of how TP-replanning, K-TP, and P-TP works are as follows.
<p align="center">
    <img src="Structure images/tp-replan structure.png" width="600">
    <img src="Structure images/tp-k structure.png" width="600">
    <img src="Structure images/tp-p structure.png" width="600">
</p>

<p align="justify">
Please note that the yellow parts that are highlighted are the only differences between the three algorithms. All three of them bases on a same algorithm of token passing, where agents passes around a token that has all of the information they need to plan paths, pickup and deliver tasks, at the same time, not collide with each other.
Tokens contain all the information of all agents' current location, task distribution, and future path plans.

As the structure shows, TP-replanning only allows agents to replan their paths after there is a collision.
However, agents in K-TP plans k amount of plans in the beginning so that they do not have to require a token when collision happens.
Lastly, P-TP has a set amount of threshold where agents require to replan if their path they are currently taking will collide with the probability above the value p.
</p>

## Simulate each algorithms
### TP-Replanning
1. Choose an environment and change the parameter 'input_name' of config.json file into the name of the environment file.
2. Modify run configuration of demo_JH.py file and add the following.
    ```
   -not_rand
    ```
3. Run demo_JH.py

### K-TP
1. Choose an environment and change the parameter 'input_name' of config.json file into the name of the environment file.
2. Modify run configuration of demo_JH.py file and add the following.
    ```
   -not_rand
    ```
3. Add another configuration for k value as the following. You can change the integer to indicate different values of k. (integer (k >= 0))
    ```
   - k 1
    ```
    ```
   - k 2
    ```
3. Run demo_JH.py

### P-TP
1. Choose an environment and change the parameter 'input_name' of config.json file into the name of the environment file.
2. Modify run configuration of demo_JH.py file and add the following.
    ```
   -not_rand
    ```
3. Add another configuration for p value as the following. You can change the float to indicate different values of p. (a float (0 <= p <= 1))
    ```
   - p 0.5
    ```
    ```
   - p 0.1
    ```
3. Run demo_JH.py

### Other command lines
1. an integer (slow_factor >= 1, default 1) which allows to slow down the visualization
    ```
    -slow_factor
    ```


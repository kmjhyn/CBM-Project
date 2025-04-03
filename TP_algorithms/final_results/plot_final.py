import yaml
import matplotlib.pyplot as plt
import numpy as np

def plot_yaml_data(file_paths):
    # Define the labels for the X-axis
    config_labels = ['tp', 'k=1', 'k=2', 'p=0.1', 'p=0.5']
    task_labels = ['total_t','task0', 'task1', 'task2']
    metrics_labels = ['n_replans', 'cost']
    colors = ['green', 'mediumblue', 'skyblue', 'mediumpurple', 'indigo']  # Define colors for each config_label

    # Read and extract data from YAML files
    completed_task_times = {config_label: {task_label: 0 for task_label in task_labels} for config_label in config_labels}
    metrics_data = {metric: {config_label: 0 for config_label in config_labels} for metric in metrics_labels}

    for i, file_path in enumerate(file_paths):
        with open(file_path, 'r') as file:
            data = yaml.load(file, Loader=yaml.FullLoader)
            for task_label in task_labels:
                completed_task_times[config_labels[i]][task_label] = data['completed_tasks_times'].get(task_label, 0)
            metrics_data['n_replans'][config_labels[i]] = data['n_replans']
            metrics_data['cost'][config_labels[i]] = data['cost']

    # Plotting
    fig, axs = plt.subplots(1, 2, figsize=(14, 6))
    # First plot: Total Time of Each Task
    ind = np.arange(len(task_labels))  # the label locations
    width = 0.15  # the width of the bars

    for i, (config_label, color) in enumerate(zip(config_labels, colors)):
        times = [completed_task_times[config_label][task_label] for task_label in task_labels]
        bars = axs[0].bar(ind + i*width, times, width, label=config_label, color=color)
        for bar in bars:
            height = bar.get_height()
            axs[0].annotate(f'{height}',
                            xy=(bar.get_x() + bar.get_width() / 2, height),
                            xytext=(0, 3),
                            textcoords="offset points",
                            ha='center', va='bottom')

    axs[0].set_title('Comparing Total Time of Each Task')
    axs[0].set_xticks(ind + width * (len(config_labels) - 1) / 2)
    axs[0].set_xticklabels(task_labels)
    axs[0].set_xlabel('Tasks')
    axs[0].set_ylabel('Completed Task Times')
    axs[0].legend()

    ind = np.arange(len(metrics_labels))  # the label locations for the metrics
    for i, metric in enumerate(metrics_labels):
        for j, (config_label, color) in enumerate(zip(config_labels, colors)):
            value = metrics_data[metric][config_label]
            bar = axs[1].bar(ind[i] + j * (width), value, width, color=color)
            axs[1].text(ind[i] + j * (width), value, f'{value}',
                        ha='center', va='bottom')

    axs[1].set_title('Comparing Number of Replans and Overall Cost by Configuration')
    axs[1].set_xticks(ind + width * (len(config_labels) - 1) / 2)
    axs[1].set_xticklabels(metrics_labels)
    axs[1].set_xlabel('Metrics')
    axs[1].set_ylabel('Value')

    # Creating a custom legend for the second plot
    custom_legend = [plt.Rectangle((0, 0), 1, 1, color=color, label=config_label) for config_label, color in zip(config_labels, colors)]
    axs[1].legend(handles=custom_legend, title='Configuration')

    plt.tight_layout()
    plt.show()

# Example usage of the function
big_complex = [
    'big_complex/output_complex_big_tp.yaml',
    'big_complex/output_complex_big_k=1.yaml',
    'big_complex/output_complex_big_k=2.yaml',
    'big_complex/output_complex_big_p=0.1.yaml',
    'big_complex/output_complex_big_p=0.5.yaml'
]

big_simple = [
    'big_simple/output_simple_big_tp.yaml',
    'big_simple/output_simple_big_k=1.yaml',
    'big_simple/output_simple_big_k=2.yaml',
    'big_simple/output_simple_big_p=0.1.yaml',
    'big_simple/output_simple_big_p=0.5.yaml'
]

small_complex = [
    'small_complex/output_complex_small_tp.yaml',
    'small_complex/output_complex_small_k=1.yaml',
    'small_complex/output_complex_small_k=2.yaml',
    'small_complex/output_complex_small_p=0.1.yaml',
    'small_complex/output_complex_small_p=0.5.yaml'
]

small_simple = [
    'small_simple/output_simple_small_tp.yaml',
    'small_simple/output_simple_small_k=1.yaml',
    'small_simple/output_simple_small_k=2.yaml',
    'small_simple/output_simple_small_p=0.1.yaml',
    'small_simple/output_simple_small_p=0.5.yaml'
]

print("big_complex")
plot_yaml_data(big_complex)

print("big_simple")
plot_yaml_data(big_simple)

print("small_complex")
plot_yaml_data(small_complex)

print("small_simple")
plot_yaml_data(small_simple)
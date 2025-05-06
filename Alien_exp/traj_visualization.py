#!/usr/bin/env python3
import json
import re
import argparse
import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator

class TrajectoryVisualizer:
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.go1_positions = {'x': [], 'y': []}
        self.go2_positions = {'x': [], 'y': []}
        self.obstacles = []

    # def plot_init(self):
    #     # Setup the plot area
    #     x_major_locator = MultipleLocator(1)
    #     self.ax.xaxis.set_major_locator(x_major_locator)
    #     self.ax.set_xlim(-2.0, 3)
    #     self.ax.set_ylim(-1, 3)
    #     self.ax.set_xlabel('X Coordinate')
    #     self.ax.set_ylabel('Y Coordinate')
    #     self.ax.grid(True)
        
    #     # Origin marker
    #     self.ax.plot(0, 0, 'ko', markersize=10, label='Origin')
        
    #     return self.ax
    
    def plot_init(self):
        # Setup the plot area
        x_major_locator = MultipleLocator(1)
        self.ax.xaxis.set_major_locator(x_major_locator)
        self.ax.set_xlim(-2.0, 3.5)
        self.ax.set_ylim(-1, 3)
        self.ax.set_xlabel('X Coordinate')
        self.ax.set_ylabel('Y Coordinate')
        self.ax.grid(True)
        
        # Create coordinate indicator boxes
        box_width = 0.8  # Size of indicator boxes
        box_height = 0.5
        # Origin box (point at origin)
        origin_box = plt.Rectangle(
            (-box_width/2, -box_height/2),  # Bottom left corner
            box_width,  # Width
            box_height,  # Height
            color='black',
            alpha=0.8,
            label='Origin'
        )
        self.ax.add_patch(origin_box)
        
        # X-direction box (placed along positive x-axis)
        x_box = plt.Rectangle(
            (box_width/2, -box_height/2),  # Bottom left corner
            box_width,  # Width
            box_height,  # Height
            color='red',
            alpha=0.8,
            label='X Direction'
        )
        self.ax.add_patch(x_box)
        
        # Y-direction box (placed along positive y-axis)
        y_box = plt.Rectangle(
            (-box_width/2, box_height/2),  # Bottom left corner
            box_width,  # Width
            box_height,  # Height
            color='green',
            alpha=0.8,
            label='Y Direction'
        )
        self.ax.add_patch(y_box)
        
        obstacle_box = plt.Rectangle(
            (-box_width/2, box_height/2),  # Bottom left corner
            box_width,  # Width
            box_height,  # Height
            color='green',
            alpha=0.8,
            label='Y Direction'
        )
        self.ax.add_patch(y_box)
        
        # Add text labels for the coordinate boxes
        self.ax.text(0, -box_width, 'Origin', horizontalalignment='center', fontsize=9)
        self.ax.text(1.0*box_width, -box_height, '+X', horizontalalignment='center', fontsize=9)
        self.ax.text(-box_width, 1.0*box_height, '+Y', horizontalalignment='center', fontsize=9)
        
        return self.ax
    
    def parse_log_file(self, log_file_path):
        with open(log_file_path, 'r') as file:
            log_content = file.readlines()
        
        for line in log_content:
            # Look for msg_str JSON data
            if 'msg_str:' in line:
                # Extract JSON part
                json_start = line.find('msg_str:') + len('msg_str:')
                json_str = line[json_start:]
                
                try:
                    data = json.loads(json_str)
                    
                    # Extract Go1 position if available
                    if 'robots' in data and 'Go1' in data['robots']:
                        go1 = data['robots']['Go1']
                        self.go1_positions['x'].append(go1['position'][0])
                        self.go1_positions['y'].append(go1['position'][1])
                    
                    # Extract Go2 position if available
                    if 'robots' in data and 'Go2' in data['robots']:
                        go2 = data['robots']['Go2']
                        self.go2_positions['x'].append(go2['position'][0])
                        self.go2_positions['y'].append(go2['position'][1])
                    
                    # Extract obstacle positions (first occurrence only)
                    if 'robots' in data and len(self.obstacles) == 0:
                        for key, value in data['robots'].items():
                            if 'box_obstacle' in key:
                                self.obstacles.append({
                                    'name': key,
                                    'x': value['position'][0],
                                    'y': value['position'][1]
                                })
                except json.JSONDecodeError:
                    print(f"Failed to parse JSON: {json_str}")
                except Exception as e:
                    print(f"Error processing line: {e}")

    def visualize_trajectory(self):
        # Plot Go1 trajectory
        if self.go1_positions['x'] and self.go1_positions['y']:
            self.ax.plot(self.go1_positions['x'], self.go1_positions['y'], 'r-', linewidth=2, label='Go1 Trajectory')
            self.ax.plot(self.go1_positions['x'][0], self.go1_positions['y'][0], 'ro', markersize=8, label='Go1 Start')
            self.ax.plot(self.go1_positions['x'][-1], self.go1_positions['y'][-1], 'rx', markersize=8, label='Go1 End')
        
        # Plot Go2 trajectory
        if self.go2_positions['x'] and self.go2_positions['y']:
            self.ax.plot(self.go2_positions['x'], self.go2_positions['y'], 'b-', linewidth=2, label='Go2 Trajectory')
            self.ax.plot(self.go2_positions['x'][0], self.go2_positions['y'][0], 'bo', markersize=8, label='Go2 Start')
            self.ax.plot(self.go2_positions['x'][-1], self.go2_positions['y'][-1], 'bx', markersize=8, label='Go2 End')
        
        # Plot obstacles
        # for obstacle in self.obstacles:
        #     self.ax.plot(obstacle['x'], obstacle['y'], 'ks', markersize=8, label=obstacle['name'])
        
        # Plot obstacles with size representation
        for obstacle in self.obstacles:
            # Use a rectangle patch instead of a point marker to show obstacle size
            obstacle_size = 0.6  # Define size in meters - adjust based on actual obstacle size
            obstacle_width = 0.6
            obstacle_height = 0.5
            obstacle_rect = plt.Rectangle(
                (obstacle['x'] - obstacle_size/2, obstacle['y'] - obstacle_size/2),  # Bottom left corner
                obstacle_width,  # Width
                obstacle_height,  # Height
                color='gray',
                alpha=0.7,
                label=obstacle['name'] if 'name' in obstacle else None
            )
            self.ax.add_patch(obstacle_rect)
            
            # Add obstacle label
            self.ax.text(obstacle['x'], obstacle['y'] + obstacle_size/2 + 0.1, 
                        obstacle['name'], 
                        horizontalalignment='center',
                        fontsize=9)
            
        obstacle_width = 0.6
        obstacle_height = 0.5
        obstacle_rect = plt.Rectangle(
            (1.0, 1.0),  # Bottom left corner
            obstacle_height,  # Width
            obstacle_width,  # Height
            color='gray',
            alpha=0.7,
            label=obstacle['name'] if 'name' in obstacle else None
        )
        self.ax.add_patch(obstacle_rect)
        
        # Add obstacle label
        self.ax.text(obstacle['x'], obstacle['y'] + obstacle_size/2 + 0.1, 
                    obstacle['name'], 
                    horizontalalignment='center',
                    fontsize=9)
        
        # Plot target if found in the log (example from your log)
        # Extract from line like "Target world: (1.8, 0.6), yaw: 1.57"
        # Here you could extend the parser to find these lines
        
        # Add legend and title
        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        self.ax.legend(by_label.values(), by_label.keys(), loc='best')
        plt.title('Robot Trajectory Visualization')
        
        plt.tight_layout()
        plt.show()

def main(args):
    visualizer = TrajectoryVisualizer()
    log_file_path = args.log_path  # Update with your log file path
    
    visualizer.plot_init()
    visualizer.parse_log_file(log_file_path)
    visualizer.visualize_trajectory()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--log_path",
        type=str,
        default=None,
        help="The path for pkl file",
    )

    args = parser.parse_args()
    try:
        main(args)
    except Exception as e:
        print(f"Error: {e}")
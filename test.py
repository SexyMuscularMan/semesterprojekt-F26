import asyncio
import time
import matplotlib.pyplot as plt
from asyncua import Client

async def main():
    # 1. Replace with your actual physical PLC IP address
    url = "opc.tcp://192.168.8.101:4841" 
    
    print("Connecting to B&R PLC OPC UA Server...")
    async with Client(url=url) as client:
        print("Connected! Locating variables...")
        
        # 2. Get the node points. (Update program name string to match your project)
        data_node = client.get_node("ns=6;s=::Prog_1ms:data")
        counter_node = client.get_node("ns=6;s=::Prog_1ms:iterationCounter")
        
        # Lists to hold data for graphing
        timestamps = []
        angles = []
        positions = []
        
        print("Starting 10-second data collection window...")
        start_time = time.time()
        
        while (time.time() - start_time) < 30.0:
            # Read the [0..3] array and the iteration counter in parallel
            physics_data = await data_node.read_value()
            current_iteration = await counter_node.read_value()
            
            # Record the current elapsed time
            elapsed = time.time() - start_time
            
            # Index mappings based on your layout:
            # physics_data[0] = cartPos
            # physics_data[1] = cartSpeed
            # physics_data[2] = angle (in rads)
            # physics_data[3] = angleVel
            current_angle_rad = physics_data[2]
            current_pos_m = physics_data[0]
            
            # Save measurements
            timestamps.append(elapsed)
            angles.append(current_angle_rad)
            positions.append(current_pos_m)
            
            # Small delay to prevent slamming the PC's CPU thread
            # 0.005s (5ms) gives a clean high-resolution trace over the network
            await asyncio.sleep(0.005)
            
        print("Data collection complete. Generating graph...")
	
        if len(angles) > 0:
           print("max angle value: ", max(angles))
           print("min angle value: ", min(angles))
           print("avg angle value: ", sum(angles) / len(angles))
        else:
           print("Error: No data was collected!")
        if len(positions) > 0:
           print("max pos value: ", max(positions))
           print("min pos value: ", min(positions))
           print("avg pos value: ", sum(positions) / len(positions))
        else:
           print("Error: No data was collected!")
        # 3. Generate the Matplotlib Plot
        #plt.figure(figsize=(10, 5))
        #plt.plot(timestamps, angles, label="Pendulum Angle (rad)", color="blue", linewidth=1.5)
       # plt.plot(timestamps, positions, label="Cart Position (m)", color="red", linewidth=1.5)

       # plt.title("Pendulum Angle and Cart Position (30 Seconds)")
       # plt.xlabel("Time (Seconds)")
       # plt.ylabel("Angle (Radians)")
       # plt.label("Cart Position (meters)")
       # plt.grid(True, linestyle="--", alpha=0.6)
       # plt.legend(loc="upper right")
        
        # Display the graph window on your desktop
       # plt.show()


# Create the main figure and primary y-axis (Left) for Angles
        fig, ax1 = plt.subplots(figsize=(10, 5))

# Plot Pendulum Angle on the left axis
        color_angle = "blue"
        ax1.plot(timestamps, angles, label="Pendulum Angle (rad)", color=color_angle, linewidth=1.5)
        ax1.set_xlabel("Time (Seconds)")
        ax1.set_ylabel("Angle (Radians)", color=color_angle)
        ax1.tick_params(axis='y', labelcolor=color_angle)

# Dynamically scale left axis to the exact data bounds (forces max/min to edges)
        if angles:
            ax1.set_ylim(min(angles), max(angles))

# Create a secondary y-axis (Right) that shares the same x-axis
        ax2 = ax1.twinx()

# Plot Cart Position on the right axis
        color_pos = "orange"
        ax2.plot(timestamps, positions, label="Cart Position (m)", color=color_pos, linewidth=1.5)
        ax2.set_ylabel("Cart Position (Meters)", color=color_pos)
        ax2.tick_params(axis='y', labelcolor=color_pos)

# Dynamically scale right axis to the exact data bounds (forces max/min to edges)
        if positions:
            ax2.set_ylim(min(positions), max(positions))

# Unified Layout Settings
        plt.title("Pendulum Angle and Cart Position (30 Seconds))")
        ax1.grid(True, linestyle="--", alpha=0.6)

# Combine legends from both axes into a single legend block
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper right")

        fig.tight_layout()  # Adjusts margins so labels don't clip at the edges
        plt.show()

if __name__ == "__main__":
    asyncio.run(main())
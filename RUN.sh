cd docker

roscore &
docker-compose up &
roslaunch --wait velodyne_pointcloud VLP16_points.launch &
roslaunch --wait zed_wrapper zed.launch

# Capture the process IDs (PIDs) of the background commands
pid1=$!
pid2=$!
pid3=$!
pid4=$!

# Function to kill all background processes
kill_background_processes() {
  # Send a SIGINT signal to each process
  kill -INT $pid1
  kill -INT $pid2
  kill -INT $pid3
  kill -INT $pid4
}

# Trap the SIGINT signal (Ctrl+C) and call the function to kill processes
trap 'kill_background_processes' INT

# Wait for all background processes to finish
wait

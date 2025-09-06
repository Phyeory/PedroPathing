#!/bin/bash

# Navigate to the flask app directory
cd "$(dirname "$0")"

# Check if virtual environment exists, if not create it
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
source venv/bin/activate

# Install requirements
echo "Installing requirements..."
pip install -r requirements.txt

# Run the Flask application
echo "Starting Flask application..."
echo "Open your browser and go to http://localhost:5000"
python app.py

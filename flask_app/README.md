# Bezier Curve Optimizer - Flask Web Application

A beautiful, modern web interface for the PedroPathing Bezier curve optimization algorithm. This application allows you to interactively design and optimize Bezier curves that avoid obstacles while maintaining smooth motion characteristics.

## Features

- **Interactive Control Point Editor**: Add, remove, and modify control points with a user-friendly interface
- **Obstacle Configuration**: Place obstacles that the curve will automatically avoid
- **Real-time Optimization**: Advanced optimization algorithms that balance curve smoothness with obstacle avoidance
- **Beautiful Visualization**: Dark-themed, responsive design with smooth animations
- **Parameter Tuning**: Adjust penalty multipliers and robot mass to fine-tune the optimization
- **Mobile Responsive**: Works great on desktop, tablet, and mobile devices

## Technology Stack

- **Backend**: Flask (Python)
- **Optimization**: SciPy optimization algorithms
- **Visualization**: Matplotlib with custom styling
- **Frontend**: Pure HTML/CSS/JavaScript with modern design
- **Mathematical Engine**: NumPy for efficient calculations

## Installation & Setup

### Method 1: Quick Start (Recommended)

1. Navigate to the flask_app directory:
   ```bash
   cd flask_app
   ```

2. Run the startup script:
   ```bash
   ./run.sh
   ```

   This script will automatically:
   - Create a virtual environment
   - Install all dependencies
   - Start the Flask application

### Method 2: Manual Setup

1. Create a virtual environment:
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Run the application:
   ```bash
   python app.py
   ```

## Usage

1. Open your web browser and navigate to `http://localhost:5000`

2. **Configure Control Points**:
   - The start and end points are fixed endpoints of your curve
   - Add intermediate control points to shape the curve
   - Modify coordinates by typing in the input fields

3. **Add Obstacles**:
   - Click "Add Obstacle" to place obstacles in the path
   - Enter X,Y coordinates for each obstacle
   - The optimizer will automatically avoid these areas

4. **Adjust Parameters**:
   - **Penalty Multiplier**: Controls how aggressively the curve avoids obstacles
   - **Robot Mass**: Affects the physical constraints of the optimization

5. **Generate Curve**:
   - Click "Generate Optimized Curve" to run the optimization
   - The visualization will show both the original and optimized curves
   - Obstacles are displayed as orange circles

## Algorithm Details

The optimization algorithm uses a physics-based approach:

- **Lagrangian Mechanics**: Balances kinetic and potential energy
- **Potential Fields**: Creates repulsive forces around obstacles
- **Curvature Minimization**: Ensures smooth, robot-friendly paths
- **SciPy Optimization**: Uses Nelder-Mead method for robust convergence

## Customization

### Styling
The application uses a modern dark theme with:
- Gradient backgrounds and glass-morphism effects
- Smooth animations and transitions
- Responsive grid layouts
- Custom sliders and form elements

### Parameters
You can modify optimization parameters in `app.py`:
- Adjust the number of sampling points for optimization
- Change the potential field strength and radius
- Modify convergence criteria and iteration limits

## File Structure

```
flask_app/
├── app.py                 # Main Flask application
├── requirements.txt       # Python dependencies
├── run.sh                # Quick start script
├── README.md             # This file
└── templates/
    └── index.html        # Main web interface
```

## Performance Notes

- The optimization is computationally intensive and may take a few seconds
- For better web performance, the sampling resolution is reduced compared to the original script
- The application uses matplotlib's non-interactive backend for server-side rendering

## Browser Compatibility

- Chrome/Chromium (recommended)
- Firefox
- Safari
- Edge

## Troubleshooting

### Common Issues

1. **Port already in use**: Change the port in `app.py` or kill the existing process
2. **Dependencies not found**: Ensure you're in the virtual environment
3. **Optimization fails**: Check that control points form a valid curve (at least 3 points)

### Performance Issues

- Reduce the number of control points for faster optimization
- Lower the penalty multiplier for quicker convergence
- Ensure obstacles are not placed too close to the curve endpoints

## Contributing

Feel free to contribute improvements:
- Enhanced visualization options
- Additional optimization algorithms
- Performance optimizations
- UI/UX improvements

## License

This project is part of the PedroPathing library and follows the same license terms.

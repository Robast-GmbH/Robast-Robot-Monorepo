import PropTypes from 'prop-types'
import SimpleMap from './SimpleMap';
import MapPositions from './MapPositions';
import Button from '@mui/material/Button';




export default function RobotControl({mapPositions, sendGoal, addPosition, StartRobot, StopRobot}) {
  return (
            <div id = "robotControl">
              <SimpleMap/>
              {mapPositions.length > 0 ? (
                <MapPositions mapPositions={mapPositions} sendGoal={sendGoal}/>
              ):(
                'No map positions set yet'
              )}
              
              <Button id="add_Position" variant="contained" onClick={addPosition} >add_Position</Button>
              <Button id="Pause" variant="contained" onClick={StopRobot}>Pause</Button>
              <Button id="Resume" variant="contained" onClick={StartRobot} >Resume</Button>

            </div>
          );
}
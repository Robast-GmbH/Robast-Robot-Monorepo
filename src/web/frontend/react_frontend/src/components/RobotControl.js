import PropTypes from 'prop-types'
import SimpleMap from './SimpleMap';
import MapPositions from './MapPositions';


export default function RobotControl({mapPositions,sendGoal}) {
    return (
              <div id = "robotControl">
                  <SimpleMap/>
                  
                  {mapPositions.length > 0 ? (
              <MapPositions mapPositions={mapPositions} sendGoal={sendGoal}/>
              ):(
                'No map positions set yet'
              )
        }
              </div>
    );

  }
import PropTypes from 'prop-types'
import SimpleMap from './SimpleMap';
import CallRobot from './CallRobot';


export default function RobotControl() {
    return (
              <div id = "robotControl">
                  <SimpleMap/>
                  <CallRobot/>
              </div>
    );

  }
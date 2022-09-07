import DisplayAnImage from './DisplayAnImage'
import {StyleSheet } from 'react-native';
import R1 from './../imgs/RobastR1.png'
import PropTypes from 'prop-types'


function RobastRobotFront( ) {
        return (
                <div id="RobastRobotFront">
                      <DisplayAnImage logo = {R1}/>  
                </div>
        )
}

RobastRobotFront.propTypes = {}

export default RobastRobotFront
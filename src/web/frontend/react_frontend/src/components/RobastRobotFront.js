import DisplayAnImage from './DisplayAnImage'
import {StyleSheet } from 'react-native';
import R1 from './../imgs/RobastR1.png'
import PropTypes from 'prop-types'
import { useTheme } from '@mui/material/styles';
import useMediaQuery from '@mui/material/useMediaQuery';


function RobastRobotFront( ) {
        const theme = useTheme();
        const matches = useMediaQuery(theme.breakpoints.up('sm'));

        if(matches)
        {
                return (
                        <div id="RobastRobotFront">
                              <img src={R1} draggable="false" alt="Robot_image"/>  
                        </div>
                )
        }
        else
        {
                return <></>
        }
}

RobastRobotFront.propTypes = {}

export default RobastRobotFront
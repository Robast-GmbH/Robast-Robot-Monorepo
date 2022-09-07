import DisplayAnImage from './DisplayAnImage'
import {StyleSheet } from 'react-native';
import TempMap from '../imgs/temp_quadrat.png'
import PropTypes from 'prop-types'





function SimpleMap( ) {
        return (
                <div id="SimpleMap">
                      <DisplayAnImage logo = {TempMap} />  
                </div>
        )
}

SimpleMap.propTypes = {}

export default SimpleMap
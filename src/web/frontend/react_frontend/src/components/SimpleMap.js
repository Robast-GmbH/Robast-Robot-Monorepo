import DisplayAnImage from './DisplayAnImage'
import {StyleSheet } from 'react-native';
import TempMap from '../imgs/temp_quadrat.png'
import PropTypes from 'prop-types'



const styles = StyleSheet.create({
        container: {
                paddingTop: 10,
              },
              tinyLogo: {
                width: 1500,
                height: 500,
              },
});

function SimpleMap( ) {
        return (
                <div id="SimpleMap">
                      <DisplayAnImage logo = {TempMap} styleIn={styles}/>  
                </div>
        )
}

SimpleMap.propTypes = {}

export default SimpleMap
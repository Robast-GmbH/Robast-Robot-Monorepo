import DisplayAnImage from './DisplayAnImage'
import {StyleSheet } from 'react-native';
import TipluOg from '../imgs/TipluOg.png'
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

function RosMap( {onClick}) {
        return (
                <div id="RosMap">
                      <DisplayAnImage onClick={onClick} logo = {TipluOg}/>  
                </div>
        )
}

RosMap.propTypes = {
        onClick: PropTypes.func,
}

export default RosMap

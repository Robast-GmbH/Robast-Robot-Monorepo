import React from 'react'
import MapPosition from './MapPosition'
import PropTypes from 'prop-types'
import ButtonGroup from '@mui/material/ButtonGroup';
import Button from './Button';


const MapPositions = ({mapPositions, sendGoal}) => {
  
  return (
    <div id="mapPositions">
      {mapPositions.map((mapPositions) => (
        
        <MapPosition mapPosition={mapPositions} sendGoal={sendGoal} />
        ))}
    </div>
    
  )
}

MapPositions.propTypes = {
    mapPositions: PropTypes.array,
    sendGoal: PropTypes.func
}


export default MapPositions
import React from 'react'
import MapPosition from './MapPosition'
import PropTypes from 'prop-types'
import ButtonGroup from '@mui/material/ButtonGroup';
import Button from './Button';


const MapPositions = ({mapPositions, sendGoal}) => {
  
  return (
    <div>
      <ButtonGroup variant="contained" aria-label="outlined primary button group">
        {mapPositions.map((mapPositions) => (
        
          <MapPosition mapPosition={mapPositions} sendGoal={sendGoal} />
        ))}
      </ButtonGroup>
    </div>
  )
}

MapPositions.propTypes = {
    mapPositions: PropTypes.array,
    sendGoal: PropTypes.func
}


export default MapPositions
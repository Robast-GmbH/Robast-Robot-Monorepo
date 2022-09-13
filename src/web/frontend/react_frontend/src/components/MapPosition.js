import React from 'react'
import Button from '@mui/material/Button';
import PropTypes from 'prop-types'


const MapPosition = ({ mapPosition, sendGoal} ) => {
 // console.log(mapPosition)
  return (
        <Button id={mapPosition.id} variant="outlined" onClick={(event)=>{sendGoal(mapPosition.id)}}> {mapPosition.name}</Button>
  )
}

MapPosition.propTypes = {
  mapPosition: PropTypes.array,
  sendGoal: PropTypes.func,
}


export default MapPosition
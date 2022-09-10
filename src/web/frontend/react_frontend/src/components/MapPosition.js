import React from 'react'
import Button from '@mui/material/Button';
import PropTypes from 'prop-types'


const MapPosition = ({ mapPosition, sendGoal} ) => {
 // console.log(mapPosition)
  return (
    <div className='mapPosition'>
        <Button id={mapPosition.id} onClick={(event)=>{sendGoal(mapPosition.id)}}> {mapPosition.name}</Button>
    </div>
  )
}

MapPosition.propTypes = {
  mapPosition: PropTypes.array,
  sendGoal: PropTypes.func,
}


export default MapPosition
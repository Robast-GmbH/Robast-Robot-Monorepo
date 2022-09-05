import React from 'react'
import Button from '@mui/material/Button';

const MapPosition = ({ mapPosition, sendGoal} ) => {
  return (
    <div className='mapPositions'>
        <Button id={mapPosition.id} onclick={sendGoal}> {mapPosition.name}</Button>
    </div>
  )
}

export default MapPosition
//<Button onclick={sendGoal} id={key}> {mapPosition.name}</Button>
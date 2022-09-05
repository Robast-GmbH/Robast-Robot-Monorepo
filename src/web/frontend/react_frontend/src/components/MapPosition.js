import React from 'react'
import Button from './Button'

const MapPosition = ({mapPositions}) => {
  return (
    <div className='mapPositions'>
        <Button text={mapPositions.title}></Button>
    </div>
  )
}

export default MapPosition
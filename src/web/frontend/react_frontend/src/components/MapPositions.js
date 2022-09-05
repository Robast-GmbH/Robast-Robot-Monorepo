import React from 'react'
import MapPosition from './MapPosition'
import PropTypes from 'prop-types'


const MapPositions = ({mapPositions, sendGoal}) => {
  return (
    <>
    {mapPositions.map((mapPositions) => (
        <MapPosition key={mapPositions.id} mapPositions={mapPositions} sendGoal={sendGoal} />
    ))  
    }
    </>
  )
}

MapPositions.propTypes = {
    mapPositions: PropTypes.array,
    sendGoal: PropTypes.func,
}


export default MapPositions
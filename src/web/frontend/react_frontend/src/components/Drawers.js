import React from 'react'
import Drawer from './Drawer'
import PropTypes from 'prop-types'


const Drawers = ({drawers, user, openDrawer,toggleEmpty}) => {
  return (
    <>
      {drawers.map((drawer) => (
          <Drawer key={drawer.id} drawer = {drawer} openDrawer = {openDrawer} toggleEmpty = {toggleEmpty} user={user} />
      ))}
    </>
  )
}

Drawers.propTypes = {
    drawers: PropTypes.array,
    user:PropTypes.object,
    openDrawer: PropTypes.func,
    toggleEmpty: PropTypes.func,
}


export default Drawers
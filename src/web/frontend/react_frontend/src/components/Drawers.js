import React from 'react'
import Drawer from './Drawer'
import PropTypes from 'prop-types'
import ButtonGroup from '@mui/material/ButtonGroup';


const Drawers = ({drawers, user, openDrawer,toggleEmpty}) => {
  return (
    <>
    <div id="OpenDrawer"  aria-label="outlined primary button group" orientation="vertical">
    {drawers.map((drawer) => (
        <Drawer key={drawer.id} drawer = {drawer} openDrawer = {openDrawer} toggleEmpty = {toggleEmpty} user={user} />
    ))}
    
    </div>
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
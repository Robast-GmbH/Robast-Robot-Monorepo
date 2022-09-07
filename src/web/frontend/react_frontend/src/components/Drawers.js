import React from 'react'
import Drawer from './Drawer'
import PropTypes from 'prop-types'
import ButtonGroup from '@mui/material/ButtonGroup';


const Drawers = ({drawers, openDrawer}) => {
console.log(drawers)
  return (
    <>
    <ButtonGroup variant="contained" aria-label="outlined primary button group" orientation="vertical">
    {drawers.map((drawer) => (
        <Drawer drawer={drawer} openDrawer={openDrawer} />
    ))  
    }
    </ButtonGroup>
    </>
  )
}

Drawers.propTypes = {
    drawers: PropTypes.array,
    openDrawer: PropTypes.func,
}


export default Drawers
import PropTypes from 'prop-types'
import { FaTimes } from 'react-icons/fa'

import React from 'react'
import Button from '@mui/material/Button';

const Drawer = ({ drawer, openDrawer} ) => {
  return (
    <div className='drawer'>
        <Button id={drawer.id} onclick={(event)=>{openDrawer(drawer)}}> {drawer.name}</Button>
    </div>
  )
}

Drawer.propTypes = {
  drawer: PropTypes.array,
  openDrawer: PropTypes.func,
}


export default  Drawer
//<Button onclick={sendGoal} id={key}> {drawer.name}</Button>
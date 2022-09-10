import PropTypes from 'prop-types'
import { FaTimes } from 'react-icons/fa'
import NoDrinksIcon from '@mui/icons-material/NoDrinks';
import IconButton from '@mui/material/IconButton';
import React from 'react'
import Button from '@mui/material/Button';
import LocalBarIcon from '@mui/icons-material/LocalBar';

const Drawer = ({ drawer, openDrawer, toggleEmpty} ) => {

  

  return (
    <div className='drawer'>
        <Button class=" DrawerSelector" id={drawer.id} onClick={(event)=>{openDrawer(drawer)}}> 
          {drawer.content} 
        </Button>
        <IconButton id="empty_button" color={drawer.empty?"error": "primary"} aria-label="toggel empty state" onClick={(event)=>{toggleEmpty(drawer)}}>
        {drawer.empty? <NoDrinksIcon/> : <LocalBarIcon/>}
        </IconButton>
     
    </div>
  )
}

Drawer.propTypes = {
  drawer: PropTypes.array,
  openDrawer: PropTypes.func,
}


export default  Drawer

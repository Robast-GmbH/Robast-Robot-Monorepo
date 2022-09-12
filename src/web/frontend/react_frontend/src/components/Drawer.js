import PropTypes from 'prop-types'
import { FaTimes } from 'react-icons/fa'
import NoDrinksIcon from '@mui/icons-material/NoDrinks';
import IconButton from '@mui/material/IconButton';
import React from 'react'
import Button from '@mui/material/Button';
import LocalBarIcon from '@mui/icons-material/LocalBar';

const Drawer = ({ drawer, user, openDrawer, toggleEmpty} ) => {

  

  return (
    <div className='drawer'>
        <Button class=" DrawerSelector" id={drawer.id} onClick={(event)=>{openDrawer(drawer)}}> 
          {drawer.content} 
        </Button>

        <IconButton color={drawer.empty ? "error" : "primary"} aria-label="toggle empty state" onClick={user.admin? (event)=>{toggleEmpty(drawer)}:()=>{}}>
          {drawer.empty? <NoDrinksIcon/> : <LocalBarIcon/>}
        </IconButton>
    </div>
  )
}

Drawer.propTypes = {
  drawer: PropTypes.object,
  user: PropTypes.object,
  openDrawer: PropTypes.func,
  toggleEmpty: PropTypes.func,
}


export default  Drawer

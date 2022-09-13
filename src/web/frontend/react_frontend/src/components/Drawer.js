import PropTypes from 'prop-types'
import { FaTimes } from 'react-icons/fa'
import NoDrinksIcon from '@mui/icons-material/NoDrinks';
import IconButton from '@mui/material/IconButton';
import React from 'react'
import ButtonUnstyled from '@mui/base/ButtonUnstyled';
import LocalBarIcon from '@mui/icons-material/LocalBar';
import { styled } from '@mui/system';
import Button from '@mui/material/Button';

import { useTheme } from '@mui/material/styles';
import useMediaQuery from '@mui/material/useMediaQuery';



const Drawer = ({ drawer, user, openDrawer, toggleEmpty} ) => {
  const theme = useTheme();
  const matches = useMediaQuery(theme.breakpoints.up('sm'));
  return (
    <div className='drawer'>
        <Button className= {matches ? "DrawerButton"  : "MDrawerButton"} variant="contained" id={"drawer" + drawer.id} onClick={(event)=>{openDrawer(drawer)}}> 
          {drawer.content} 
        </Button>

        <div className="drawerStatusIcon">
          { user.admin === true ?  (<IconButton color={(drawer.empty) ? "error" : "success"} aria-label="toggle empty state" onClick= {(event)=>{toggleEmpty(drawer)}}>
            {drawer.empty? <NoDrinksIcon/> : <LocalBarIcon/>}
          </IconButton>):(drawer.empty? <NoDrinksIcon color= "error"/> : <LocalBarIcon sx={{ color:"#FFF" }}/>)}
        </div>
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

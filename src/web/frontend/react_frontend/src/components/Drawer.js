import PropTypes from 'prop-types'
import { FaTimes } from 'react-icons/fa'
import NoDrinksIcon from '@mui/icons-material/NoDrinks';
import IconButton from '@mui/material/IconButton';
import React from 'react'
import ButtonUnstyled from '@mui/base/ButtonUnstyled';
import LocalBarIcon from '@mui/icons-material/LocalBar';
import { styled } from '@mui/system';

const DrawerButton = styled(ButtonUnstyled)`
  outline: none;
  border-width: 0;
  border-radius: 66px;
  background-image: linear-gradient(to right, #1565c0, #2f7ed8, #5a8fcb, #6ca2e1);
  background-size: 300% 100%;
  color: #fff;
  transition: all .3s;
  font-size: 30px;
  box-sizing: border-box;
  padding: 20px 40px;
  flex-grow: 1;
  cursor: pointer;
  margin-right: 10px;
  user-select: none;

  &:hover {
    background-position: 100% 0;
  }

  &:active {
    background-position: 100% 0;
  }
`;


const Drawer = ({ drawer, user, openDrawer, toggleEmpty} ) => {
  return (
    <div className='drawer'>
        <DrawerButton class="DrawerSelector" id={"drawer" + drawer.id} onClick={(event)=>{openDrawer(drawer)}}> 
          {drawer.content} 
        </DrawerButton>

        {user.admin===true? (<IconButton color={drawer.empty ? "error" : "primary"} aria-label="toggle empty state" onClick= {(event)=>{toggleEmpty(drawer)}}>
          {drawer.empty? <NoDrinksIcon/> : <LocalBarIcon/>}
        </IconButton>):(drawer.empty? <NoDrinksIcon color= "error"/> : <LocalBarIcon sx={{ color:"#FFF" }}/>)}
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

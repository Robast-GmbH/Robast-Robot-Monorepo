import React from 'react'
import Avatar from '@mui/material/Avatar';
import Stack from '@mui/material/Stack';
import Button from '@mui/material/Button';
import ButtonPopUp from './ButtonPopup';
import Login from './Login';


const stringAvatar = (name) => {
    return {
      sx: {
        bgcolor: stringToColor(name),
      },
      children: `${name.split(' ')[0][0]}${name.split(' ')[1][0]}`,
    };
  }

const stringToColor = (string) => {
    let hash = 0;
    let i;
  
    /* eslint-disable no-bitwise */
    for (i = 0; i < string.length; i += 1) {
      hash = string.charCodeAt(i) + ((hash << 5) - hash);
    }
  
    let color = '#';
  
    for (i = 0; i < 3; i += 1) {
      const value = (hash >> (i * 8)) & 0xff;
      color += `00${value.toString(16)}`.slice(-2);
    }
    /* eslint-enable no-bitwise */
  
    return color;
}

const loginPopupModal= (login)=> {
  return ( 
    <Login  login= {login}/>
    ); }

const userDisplay= ({logout, user})=>
{
  console.log(user)
    return( 
        <Stack spacing={2} direction="row">
            <Avatar {...stringAvatar(user.full_name)}/> 
            <Button id="logOut" variant="text" size="small" onClick={logout}>Logout</Button>
        </Stack>
    );
}

const loginDisplay = ({login})=>
{
  return(
      <ButtonPopUp caption="Admin Login" key="login_button" name="login" popUp={loginPopupModal(login)} />
  )
}

const UserManagement = ({user, login, logout}) => {
    
    //const token = sessionStorage.getItem('token')
    
    return (
      <>
        {user.id === 0 ?(loginDisplay({login})):(userDisplay({user, logout} ))}
      </>
    )
}

export default UserManagement
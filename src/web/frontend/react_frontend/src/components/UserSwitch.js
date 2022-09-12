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
      children: `${name.split(' ')[0][0]}`,
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
  <div>
    <h1> Login </h1>
    {<Login  login= {login}/>}
  </div>
    ); }

const userDisplay= ({logout, user})=>
{
    return( 
        <Stack spacing={2} direction="row">
            <Avatar {...stringAvatar(user.name)}/> 
            <Button id="logOut" variant="text" size="small" onClick={logout}>Logout</Button>
        </Stack>
    );
}

const loginDisplay = ({login})=>
{
  return(
      <ButtonPopUp caption="Login" key="login_button" name="login" popUp={loginPopupModal(login)} />
  )
}

const UserSwitch = ({user, login, logout}) => {
    
    const token = sessionStorage.getItem('token')
    user.name="test"
    return (
      <>
        {user.id == 0 ?(loginDisplay({login})):(userDisplay({user, logout} ))}
      </>
    )
}

export default UserSwitch
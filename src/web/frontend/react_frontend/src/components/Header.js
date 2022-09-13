import Button from './Button'
import { useLocation } from 'react-router-dom'
import DisplayAnImage from './DisplayAnImage';
import Logo from '../imgs/Logo.png';
import UserManagement from './UserManagement';


const Header = ({user, login, logout}) => {

        const location = useLocation()
        return (
                <header className='header'>
                        
                        <DisplayAnImage logo={Logo}></DisplayAnImage>
                        <UserManagement user = {user} login={login} logout={logout} />
                        
                </header>
        )
}

Header.defaultProps = {
        lable: 'Robast Logo',
}

export default Header